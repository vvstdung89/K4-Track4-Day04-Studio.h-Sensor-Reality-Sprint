"""Per-scan log of the E5 failure case (cruise 40 s, then -6 m/s^2 braking), same seed as fig4 (seed 1).

  results/e5_failure_log_seed1.csv          every radar scan, all four methods
  results/e5_failure_log_excerpt.txt        one scan per second around brake onset (t = 30 ... 46 s)

Run:  python export_failure_log.py   (about 10 s)
"""
import os

import numpy as np

import rio_tc_sim as R

SEED = 1                                                  # plot_e5 draws out[name][1]
CASES = [("no-TC", "no-TC", 0.0), ("TC_cold", "TC", 0.0), ("TC_warm", "TC", R.TD_PAPER), ("oracle", "oracle", 0.0)]


def main():
    sc = R.make_scenario("aeb", 50.0, lambda _t: R.TD_PAPER)
    runs = {name: R.run(sc, mode, SEED, td_init=ti) for name, mode, ti in CASES}
    t = runs["TC_cold"]["t"]
    n = min(len(r["t"]) for r in runs.values())
    speed = np.interp(t[:n], sc["t"], sc["u"])
    cols = ["t_s", "true_speed_mps", "td_true_ms"]
    data = [t[:n], speed, R.ms(runs["TC_cold"]["td_true"][:n])]
    for name in ["TC_cold", "TC_warm"]:
        r = runs[name]
        cols += [f"{name}_td_hat_ms", f"{name}_sigma_td_ms"]
        data += [R.ms(r["td"][:n]), R.ms(r["td_sig"][:n])]
    for name in runs:
        cols += [f"{name}_ego_err_mps", f"{name}_nis"]
        data += [runs[name]["ego_err"][:n], runs[name]["nis"][:n]]
    table = np.stack(data, 1)
    np.savetxt(os.path.join(R.OUT, f"e5_failure_log_seed{SEED}.csv"), table, delimiter=",",
               header=",".join(cols), comments="", fmt="%.4f")

    rows = [f"# E5 failure log, seed {SEED} (same run as fig4). t_d true = -113 ms; braking 40.0-43.3 s.",
            f"# NIS > {R.CHI2_99} = scan a 99 % gate would reject (ghost-rate proxy). Full log: e5_failure_log_seed{SEED}.csv",
            "",
            f"{'t[s]':>5} {'speed':>6} | {'td_hat':>7} {'sigma':>6} {'err/sig':>7} | "
            f"{'ego err no-TC':>13} {'TC cold':>8} {'oracle':>7} | {'NIS no-TC':>9} {'TC cold':>8}"]
    tc, no, orc = runs["TC_cold"], runs["no-TC"], runs["oracle"]
    for ts in list(range(30, 40, 2)) + [39.5, 40.0, 40.5, 41.0, 42.0, 43.0, 44.0, 46.0]:
        i = int(np.searchsorted(t, ts))
        err = tc["td"][i] - tc["td_true"][i]
        flag = lambda x: f"{x:8.1f}" + ("*" if x > R.CHI2_99 else " ")
        rows.append(f"{t[i]:5.1f} {speed[i]:6.1f} | {1000 * tc['td'][i]:7.0f} {1000 * tc['td_sig'][i]:6.1f} "
                    f"{err / tc['td_sig'][i]:7.1f} | {no['ego_err'][i]:13.3f} {tc['ego_err'][i]:8.3f} "
                    f"{orc['ego_err'][i]:7.3f} | {flag(no['nis'][i])} {flag(tc['nis'][i])}")
    rows += ["", "columns: td_hat, sigma = TC cold start [ms]; err/sig = (td_hat - td_true) / sigma; "
                 "ego err [m/s]; * = NIS above the 99 % gate"]
    text = "\n".join(rows)
    print(text)
    with open(os.path.join(R.OUT, "e5_failure_log_excerpt.txt"), "w") as fh:
        fh.write(text + "\n")


if __name__ == "__main__":
    main()
