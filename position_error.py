"""E7 - T4 minimum benchmark: position error of a radar target caused by a time offset, before/after compensation.

A radar detection is measured in the sensor frame at the capture time t_c, but it is placed in the world with the
ego pose of the time the system *thinks* it was measured:
  before compensation  tau = stamp            (= t_c - t_d, arrival time)
  after  compensation  tau = stamp + t_d_hat  (EKF-RIO-TC estimate of the same run)
  oracle               tau = t_c
The true ego pose is used for the projection, so the error is only the timing error, not odometry drift.
Expected:  position error ~= |relative velocity| x |tau - t_c|; for a static target and straight driving this is
ego speed x offset.  It is no longer exact when the ego turns (the target also swings by yaw rate x range x offset).

Parts
  P1 straight road, constant speed 5-30 m/s, offset 50-200 ms        -> check error = v x offset
  P2 city motion (E1 scenario), t_d in {-50, -113, -150, -200} ms, 10 seeds -> before / after TC / oracle
  P3 cruise 40 s then braking (E5 scenario), t_d = -113 ms, 10 seeds  -> error at brake onset

Outputs: results/position_error.md, results/fig6_timeline_position_error.png
Run:  python position_error.py   (about 1 minute)
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rio_tc_sim as R

RANGE = 30.0                                   # target range [m]
BEARINGS = np.radians([0.0, 30.0])             # straight ahead, 30 deg to the left
THRESH = {"0.5 m (association gate)": 0.5, "1.75 m (half a lane)": 1.75}
SEEDS = range(10)


def pose(sc, tau):
    x = np.interp(tau, sc["t"], sc["p"][:, 0])
    y = np.interp(tau, sc["t"], sc["p"][:, 1])
    return np.stack([x, y], -1), np.interp(tau, sc["t"], sc["th"])


def project_error(sc, tc, tau, bearing):
    """World position error of a static target seen at range RANGE, `bearing`, at capture time tc, placed with pose(tau)."""
    p_c, th_c = pose(sc, tc)
    rel = RANGE * np.stack([np.cos(th_c + bearing), np.sin(th_c + bearing)], -1)
    target = p_c + rel                                           # true world position
    z_body = np.stack([RANGE * np.cos(bearing) * np.ones_like(tc), RANGE * np.sin(bearing) * np.ones_like(tc)], -1)
    p_t, th_t = pose(sc, tau)
    c, s = np.cos(th_t), np.sin(th_t)
    placed = p_t + np.stack([c * z_body[:, 0] - s * z_body[:, 1], s * z_body[:, 0] + c * z_body[:, 1]], -1)
    return np.linalg.norm(placed - target, axis=1)


def times(sc, mode, seed, td_init=0.0):
    """Capture time, arrival stamp and the time used for the projection by each method (same scans as R.run)."""
    _, _, tc, stamp, _, _ = R.make_measurements(sc, np.random.default_rng(seed))
    if mode == "before":
        return tc, stamp, stamp
    if mode == "oracle":
        return tc, stamp, tc
    res = R.run(sc, "TC", seed, td_init=td_init)
    td_prior = np.concatenate([[td_init], res["td"][:-1]])       # estimate available when the scan arrives
    return tc, stamp, stamp + td_prior


def p1_straight():
    rows = []
    for v in [5.0, 10.0, 20.0, 30.0]:
        sc = R.make_scenario("city", 10.0, lambda _t: 0.0)
        t = sc["t"]
        sc.update(u=np.full_like(t, v), w=np.zeros_like(t), th=np.zeros_like(t),
                  v=np.stack([np.full_like(t, v), np.zeros_like(t)], 1), p=np.stack([v * t, np.zeros_like(t)], 1))
        for off in [0.05, 0.10, 0.15, 0.20]:
            tc = np.arange(1.0, 9.0, 1 / R.RADAR_HZ)
            err = project_error(sc, tc, tc + off, 0.0)
            rows.append(dict(v=v, off=off, meas=float(np.mean(err)), formula=v * off))
    return rows


def p2_city():
    rows, example = [], None
    for td in [-0.050, -0.113, -0.150, -0.200]:
        sc = R.make_scenario("city", 60.0, lambda _t, td=td: td)
        out = {}
        for mode in ["before", "TC", "oracle"]:
            errs, errs30, form, late = [], [], [], []
            for s in SEEDS:
                tc, stamp, tau = times(sc, mode, s)
                e0 = project_error(sc, tc, tau, BEARINGS[0])
                e30 = project_error(sc, tc, tau, BEARINGS[1])
                spd = np.interp(tc, sc["t"], sc["u"])
                m = tc > 10.0                                     # after the TC convergence phase
                errs.append(R.rmse(e0[m])); errs30.append(R.rmse(e30[m]))
                form.append(R.rmse((spd * np.abs(tau - tc))[m])); late.append(np.max(e0[m]))
                if td == -0.113 and s == 0:
                    example = example or {}
                    example[mode] = (tc, e0, spd * np.abs(tau - tc))
            out[mode] = dict(rmse=np.mean(errs), rmse30=np.mean(errs30), formula=np.mean(form), peak=np.mean(late))
        rows.append(dict(td=td, **{f"{k}_{m}": v for m, d in out.items() for k, v in d.items()}))
    return rows, example


def p3_cruise_brake():
    sc = R.make_scenario("aeb", 50.0, lambda _t: R.TD_PAPER)
    rows = []
    for name, mode, ti in [("before (no compensation)", "before", 0.0), ("TC cold start", "TC", 0.0),
                           ("TC warm start", "TC", R.TD_PAPER), ("oracle", "oracle", 0.0)]:
        cruise, end, brake = [], [], []
        for s in SEEDS:
            tc, stamp, tau = times(sc, mode, s, ti)
            e = project_error(sc, tc, tau, BEARINGS[0])
            ok = tau < R.T_BRAKE                                   # fused before the filter sees any braking
            cruise.append(R.rmse(e[ok & (tc > 5.0)]))
            end.append(np.mean(e[ok & (tc > R.T_BRAKE - 1.0)]))
            brake.append(R.rmse(e[(tc >= R.T_BRAKE) & (tc < R.T_BRAKE + 3.3)]))
        rows.append(dict(name=name, cruise=np.mean(cruise), end=np.mean(end), brake=np.mean(brake)))
    return rows


def plot(p1, example):
    fig, ax = plt.subplots(1, 3, figsize=(17, 4.6))
    # (a) multi-sensor timeline
    a = ax[0]
    sc = R.make_scenario("city", 60.0, lambda _t: R.TD_PAPER)
    tc, stamp, tau = times(sc, "TC", 0)
    w0, w1 = 20.0, 20.5
    imu = sc["t"][(sc["t"] >= w0) & (sc["t"] <= w1)]
    a.vlines(imu, 2.8, 3.2, color="gray", lw=0.6)
    sel = (tc >= w0) & (tc <= w1 - 0.12)
    a.plot(tc[sel], np.full(sel.sum(), 2), "o", color="tab:green", label="radar capture t_c (true)")
    a.plot(stamp[sel], np.full(sel.sum(), 1), "s", color="tab:red", label="radar stamp = arrival (t_c + 113 ms)")
    a.plot(tau[sel], np.full(sel.sum(), 0), "^", color="tab:blue", label="after compensation: stamp + t_d_hat")
    for x0, x1 in zip(tc[sel], stamp[sel]):
        a.annotate("", xy=(x1, 1.1), xytext=(x0, 1.9), arrowprops=dict(arrowstyle="->", color="tab:red", lw=0.8))
    a.set(yticks=[0, 1, 2, 3], yticklabels=["fusion time\n(TC)", "radar stamp", "radar capture", "IMU 200 Hz"],
          xlabel="time [s]", title="Multi-sensor timeline (city run, t_d = -113 ms)", xlim=(w0, w1), ylim=(-0.6, 3.6))
    a.legend(fontsize=7, loc="upper right")
    # (b) error = velocity x offset
    a = ax[1]
    offs = np.linspace(0, 0.2, 50)
    for v, c in zip([5, 10, 20, 30], ["tab:green", "tab:olive", "tab:orange", "tab:red"]):
        a.plot(1000 * offs, v * offs, color=c, lw=1, label=f"v x offset, v = {v} m/s")
        pts = [r for r in p1 if r["v"] == v]
        a.plot([1000 * r["off"] for r in pts], [r["meas"] for r in pts], "o", color=c)
    for name, y in THRESH.items():
        a.axhline(y, color="k", ls=":", lw=0.9)
        a.text(2, y + 0.08, name, fontsize=8)
    a.axvline(113, color="gray", ls="--", lw=0.8)
    a.set(xlabel="time offset [ms]", ylabel="target position error [m]",
          title="P1 straight road: line = formula, dot = measured")
    a.legend(fontsize=7, loc="upper left")
    # (c) before / after compensation, city
    a = ax[2]
    for mode, c, lab in [("before", "tab:red", "before compensation"), ("TC", "tab:blue", "after TC compensation"),
                         ("oracle", "k", "oracle")]:
        tc_, e, f = example[mode]
        a.plot(tc_, e, color=c, lw=1, label=lab)
    tc_, _, f = example["before"]
    a.plot(tc_, f, color="tab:red", ls="--", lw=0.8, label="formula speed x 113 ms")
    a.axhline(1.75, color="k", ls=":", lw=0.9)
    a.set(xlabel="time [s]", ylabel="target position error [m] (target 30 m ahead)",
          title="P2 city, t_d = -113 ms (seed 0)", xlim=(0, 60))
    a.legend(fontsize=7, loc="upper right")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(R.OUT, "fig6_timeline_position_error.png"), dpi=120)
    plt.close(fig)


def main():
    p1 = p1_straight()
    p2, example = p2_city()
    p3 = p3_cruise_brake()
    L = ["# E7 - position error from the time offset (auto-generated by position_error.py)", "",
         f"Static target at {RANGE:.0f} m, placed in the world with the true ego pose at the assumed time. Mean of 10 seeds.",
         "", "## P1 - straight road, constant speed: check error = speed x offset", "",
         "| speed [m/s] | offset [ms] | measured error [m] | formula v x offset [m] |", "|---|---|---|---|"]
    L += [f"| {r['v']:.0f} | {1000 * r['off']:.0f} | {r['meas']:.3f} | {r['formula']:.3f} |" for r in p1]
    L += ["", "## P2 - city motion (E1 scenario), after 10 s; before / after TC / oracle", "",
          "| t_d [ms] | RMSE target 0 deg: before / TC / oracle [m] | RMSE target 30 deg: before / TC / oracle [m] "
          "| formula speed x offset, before [m] | peak error before [m] |", "|---|---|---|---|---|"]
    for r in p2:
        L.append(f"| {1000 * r['td']:+.0f} | {r['rmse_before']:.2f} / {r['rmse_TC']:.3f} / {r['rmse_oracle']:.3f} "
                 f"| {r['rmse30_before']:.2f} / {r['rmse30_TC']:.3f} / {r['rmse30_oracle']:.3f} "
                 f"| {r['formula_before']:.2f} | {r['peak_before']:.2f} |")
    L += ["", "## P3 - cruise 40 s at 25 m/s, then -6 m/s^2 braking (E5 scenario), t_d = -113 ms", "",
          "| method | RMSE during cruise 5-40 s [m] | mean error in the last second of cruise [m] | RMSE during braking [m] |",
          "|---|---|---|---|"]
    L += [f"| {r['name']} | {r['cruise']:.2f} | {r['end']:.2f} | {r['brake']:.2f} |" for r in p3]
    L += ["", "## Danger threshold: largest offset that keeps the error below the threshold (straight road)", "",
          "| speed [m/s] | " + " | ".join(THRESH) + " |", "|---|" + "---|" * len(THRESH)]
    for v in [10, 20, 25, 30]:
        L.append(f"| {v} | " + " | ".join(f"{1000 * y / v:.0f} ms" for y in THRESH.values()) + " |")
    text = "\n".join(L)
    print(text)
    with open(os.path.join(R.OUT, "position_error.md"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    plot(p1, example)


if __name__ == "__main__":
    main()
