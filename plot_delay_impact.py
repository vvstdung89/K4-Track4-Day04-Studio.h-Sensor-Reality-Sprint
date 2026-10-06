"""Real-world failure: what an uncompensated radar delay does to the ego-velocity feature.

Uses outputs already in results/ (no re-run):
  results/results.md                 E1 table (10 seeds): no-TC / TC / oracle vs injected delay
  results/e5_failure_log_seed1.csv   E5 per-scan log (seed 1): cruise 40 s, then -6 m/s^2 braking
Writes results/fig5_delay_impact.png.  Run after rio_tc_sim.py and export_failure_log.py:
  python plot_delay_impact.py
"""
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


def read_e1():
    text = open(os.path.join(OUT, "results.md"), encoding="utf-8").read()
    block = text.split("## E1")[1].split("## E2")[0]
    rows = []
    for line in block.splitlines():
        m = re.match(r"\|\s*([+-]?\d+)\s*\|", line)
        if not m:
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        gf = [float(x.strip(" %")) for x in cells[3].split("/")]
        ego = [float(x) for x in cells[4].split("/")]
        rows.append((int(m.group(1)), gf, ego))
    rows.sort()
    td = np.array([r[0] for r in rows])
    return td, np.array([r[1] for r in rows]), np.array([r[2] for r in rows])


def main():
    td, gf, ego = read_e1()
    log = np.genfromtxt(os.path.join(OUT, "e5_failure_log_seed1.csv"), delimiter=",", names=True)

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.4))
    names, colors = ["no-TC (delay ignored)", "TC (paper)", "oracle"], ["tab:red", "tab:blue", "k"]
    for k in range(3):
        ax[0].plot(td, ego[:, k], "o-", color=colors[k], label=names[k])
        ax[1].plot(td, gf[:, k], "o-", color=colors[k], label=names[k])
    ax[0].set(xlabel="radar-IMU delay t_d [ms]", ylabel="ego-velocity RMSE [m/s]",
              title="E1 (10 seeds): ego-velocity error vs delay")
    ax[1].axhline(1.0, color="gray", lw=0.8, ls=":")
    ax[1].set(xlabel="radar-IMU delay t_d [ms]", ylabel="scans failing 99 % gate [%]",
              title="E1: ghost-rate proxy (NIS > 9.21) vs delay")
    for a in ax[:2]:
        a.axvline(-113, color="gray", lw=0.8, ls="--")
        a.text(-111, a.get_ylim()[1] * 0.92, "paper: -113 ms", fontsize=8, color="gray")
        a.legend(fontsize=8)

    t = log["t_s"]
    m = (t > 34) & (t < 49)
    ax[2].plot(t[m], log["noTC_ego_err_mps"][m], color="tab:red", label="no-TC ego-velocity error")
    ax[2].plot(t[m], log["oracle_ego_err_mps"][m], color="k", label="oracle ego-velocity error")
    ax[2].axvspan(40.0, 43.3, color="tab:red", alpha=0.08, label="AEB braking -6 m/s^2")
    ax[2].axhline(6 * 0.113, color="gray", lw=0.8, ls="--")
    ax[2].text(34.3, 6 * 0.113 + 0.03, "6 m/s^2 x 0.113 s = 0.68 m/s", fontsize=8, color="gray")
    ax[2].set(xlabel="time [s]", ylabel="ego-velocity error [m/s]", ylim=(0, 1.0),
              title="E5 (seed 1, t_d = -113 ms): cruise 25 m/s, then braking")
    a2 = ax[2].twinx()
    a2.plot(t[m], log["true_speed_mps"][m], color="tab:green", lw=0.8, ls="--")
    a2.set_ylabel("true speed [m/s] (dashed green)", color="tab:green")
    ax[2].legend(fontsize=8, loc="upper right")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_delay_impact.png"), dpi=120)
    plt.close(fig)
    print("wrote results/fig5_delay_impact.png")


if __name__ == "__main__":
    main()
