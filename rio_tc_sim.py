"""T4 submission - mini simulation benchmark for EKF-RIO-TC (online radar-IMU time-offset estimation).

Selected paper: Kim, Bae, Shin, Wang, Oh, "EKF-Based Radar-Inertial Odometry with Online Temporal
Calibration", IEEE RA-L 10(7), 2025 (arXiv 2502.00661, code github.com/spearwin/EKF-RIO-TC).

Planar (2-D) re-implementation of the paper's idea:
  * error-state EKF, state x = [theta, b_g, v_x, v_y, b_ax, b_ay, t_d]; position is dead-reckoned
  * radar ego-velocity measurement (body frame)  h = R(theta)^T v + (w - b_g) x p_RI,
    evaluated at t' = t + t_d_hat, where t is the radar timestamp and the IMU is the time reference
  * Jacobian column of t_d = d h / dt = H_theta * w + H_v * R a   (planar form of the paper's H_td)
  * like the paper, outliers are assumed removed by RANSAC upstream, so the filter has no gate

Methods compared
  no-TC   : EKF-RIO baseline, t_d fixed to 0 (radar fused at its arrival timestamp)
  TC      : the paper, t_d in the state, small random walk (sigma_td = 0.1 ms/sqrt(s))
  TC-bigQ : the paper with a 50x larger t_d random walk (the paper's convergence/stability trade-off)
  oracle  : true t_d known (upper bound)

Metrics (T4: offset error, ghost-rate proxy, trajectory residual)
  offset error  |t_d_hat - t_d| [ms], convergence time [s] (|error| < 10 ms from then on)
  gate-fail     fraction of radar scans whose innovation fails a 99 % chi-square test (NIS > 9.21);
                the radar-odometry analogue of a ghost / association failure
  residual      ego-velocity error in the body frame [m/s], RPE over 10 m [m] (paper metric),
                SE(2)-aligned APE [m], radar innovation RMS [m/s]

Experiments
  E1 injected-delay protocol (as the paper does on ICINS): t_d in {-200 ... +50} ms, rich "city" motion
  E2 convergence from several initial t_d values (paper Fig. 5 analogue)
  E3 excitation sweep (benchmark extension): acceleration amplitude 0 ... 4 m/s^2
  E4 limitation: t_d steps from -113 ms to -163 ms in the middle of the drive
  E5 failure case: 40 s highway cruise (no excitation), then AEB-like braking
  E6 ablation: which IMU error drives the E5 drift

Only numpy and matplotlib are needed.  Run:  python rio_tc_sim.py   (about 6 minutes)
"""
import os
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# ----------------------------------------------------------------- sensors
IMU_HZ, RADAR_HZ = 200.0, 15.0
DT = 1.0 / IMU_HZ
LEVER = np.array([3.5, 0.0])          # radar position in the IMU frame [m] (front bumper)
SIG_ACC, SIG_GYR = 0.10, 0.005        # IMU white noise per sample [m/s^2], [rad/s]
BIAS_ACC, BIAS_GYR = np.array([0.08, -0.05]), 0.003
SIG_RADAR = 0.05                      # radar ego-velocity noise per axis after RANSAC [m/s]
CHI2_99 = 9.21                        # chi-square 99 %, 2 dof (gate-fail metric only)

# ----------------------------------------------------------------- filter tuning
SIG_TD0 = 0.10                        # initial t_d std [s]
SIG_TD_RW = 1e-4                      # t_d random walk [s/sqrt(s)]  (paper-style: ~constant)
SIG_TD_RW_BIG = 5e-3                  # 50x larger (TC-bigQ)
SIG_BG_RW, SIG_BA_RW = 1e-5, 1e-4

TD_PAPER = -0.113                     # radar-IMU offset reported by the paper [s]
CONV_TOL = 0.010                      # "converged" = |t_d error| < 10 ms from then on
RPE_DIST = 10.0                       # RPE segment length [m] (paper: 10 m)
J = np.array([[0.0, -1.0], [1.0, 0.0]])
CROSS = np.array([-LEVER[1], LEVER[0]])  # w x p_RI = w * CROSS


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


# ----------------------------------------------------------------- scenario
def make_scenario(kind, T, td_fun, A=2.0):
    """Ground truth on the IMU grid (no side slip): speed u, accel du, yaw rate w, heading, velocity, position."""
    t = np.arange(0.0, T, DT)
    if kind == "city":                      # accel amplitude 2.4 m/s^2, yaw rate 0.25 rad/s
        u = 10 + 3 * np.sin(2 * np.pi * t / 8)
        w = 0.25 * np.sin(2 * np.pi * t / 11)
    elif kind == "excite":                  # accel amplitude A, yaw-rate amplitude A/10
        Pu = 6.0
        u = 15 + A * Pu / (2 * np.pi) * np.sin(2 * np.pi * t / Pu)
        w = A / 10 * np.sin(2 * np.pi * t / 9)
    elif kind == "aeb":                     # 40 s highway cruise at 25 m/s, then -6 m/s^2 braking to 5 m/s
        u = np.clip(25 - 6 * np.clip(t - T_BRAKE, 0, None), 5, None)
        w = np.zeros_like(t)
    else:
        raise ValueError(kind)
    du = np.gradient(u, DT)
    th = np.concatenate([[0.0], np.cumsum(w[:-1]) * DT])
    vw = np.stack([u * np.cos(th), u * np.sin(th)], 1)
    p = np.concatenate([[[0, 0]], np.cumsum((vw[:-1] + vw[1:]) / 2 * DT, 0)])
    dist = np.concatenate([[0.0], np.cumsum((u[:-1] + u[1:]) / 2 * DT)])
    return dict(t=t, u=u, du=du, w=w, th=th, v=vw, p=p, dist=dist, T=T, td_fun=td_fun)


def make_measurements(sc, rng):
    t, u, du, w = sc["t"], sc["u"], sc["du"], sc["w"]
    acc = np.stack([du, u * w], 1) + BIAS_ACC + rng.normal(0, SIG_ACC, (len(t), 2))
    gyr = w + BIAS_GYR + rng.normal(0, SIG_GYR, len(t))
    tc = np.arange(0.5, sc["T"] - 0.5, 1 / RADAR_HZ)                    # capture times
    uc, wc = np.interp(tc, t, u), np.interp(tc, t, w)
    z = np.stack([uc, np.zeros_like(uc)], 1) + wc[:, None] * CROSS     # body-frame ego velocity
    z += rng.normal(0, SIG_RADAR, z.shape)
    td_true = np.array([sc["td_fun"](x) for x in tc])
    stamp = tc - td_true                                                # late arrival stamp
    order = np.argsort(stamp)
    return acc, gyr, tc[order], stamp[order], z[order], td_true[order]


# ----------------------------------------------------------------- filter
class EkfRio:
    """Planar error-state EKF radar-inertial odometry with optional online t_d.

    Error state [theta, b_g, v_x, v_y, b_ax, b_ay, t_d]. Position is dead-reckoned from v outside the
    filter: radar ego-velocity never observes it, and keeping it in the state only lets the EKF's
    spurious yaw information shift it (the known EKF yaw-inconsistency), which would blur the RPE.
    """

    TD = 6

    def __init__(self, sc, mode, td_init):
        self.th, self.bg = sc["th"][0], 0.0
        self.v, self.ba, self.p = sc["v"][0].copy(), np.zeros(2), np.zeros(2)
        self.td = 0.0 if mode == "no-TC" else td_init
        est = mode.startswith("TC")
        self.P = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 0.1 ** 2, 0.1 ** 2, SIG_TD0 ** 2 if est else 0.0])
        rw = SIG_TD_RW_BIG if mode == "TC-bigQ" else SIG_TD_RW
        self.q_td = rw ** 2 if est else 0.0

    def propagate(self, a_m, g_m, dt):
        w, a, R = g_m - self.bg, a_m - self.ba, rot(self.th)
        Ra = R @ a
        F = np.zeros((7, 7))
        F[0, 1] = -1.0
        F[2:4, 0] = R @ J @ a
        F[2:4, 4:6] = -R
        Phi = np.eye(7) + F * dt
        Q = np.zeros((7, 7))
        Q[0, 0] = SIG_GYR ** 2 * DT * dt
        Q[2:4, 2:4] = SIG_ACC ** 2 * DT * dt * np.eye(2)
        Q[1, 1], Q[4, 4], Q[5, 5] = SIG_BG_RW ** 2 * dt, SIG_BA_RW ** 2 * dt, SIG_BA_RW ** 2 * dt
        Q[6, 6] = self.q_td * dt
        self.P = Phi @ self.P @ Phi.T + Q
        self.p = self.p + self.v * dt + 0.5 * Ra * dt * dt
        self.v = self.v + Ra * dt
        self.th = self.th + w * dt

    def update(self, z, a_m, g_m):
        w, a, R = g_m - self.bg, a_m - self.ba, rot(self.th)
        h = R.T @ self.v + w * CROSS
        H = np.zeros((2, 7))
        H[:, 0] = R.T @ J.T @ self.v
        H[:, 1] = -CROSS
        H[:, 2:4] = R.T
        if self.P[6, 6] > 0:
            H[:, 6] = H[:, 0] * w + a                     # = H_theta * w + H_v * R a
        S = H @ self.P @ H.T + SIG_RADAR ** 2 * np.eye(2)
        r = z - h
        Si = np.linalg.inv(S)
        nis = float(r @ Si @ r)
        K = self.P @ H.T @ Si
        dx = K @ r
        self.th += dx[0]; self.bg += dx[1]; self.v = self.v + dx[2:4]
        self.ba = self.ba + dx[4:6]; self.td += dx[6]
        IKH = np.eye(7) - K @ H
        self.P = IKH @ self.P @ IKH.T + SIG_RADAR ** 2 * K @ K.T
        return nis, float(np.linalg.norm(r))


def run(sc, mode, seed, td_init=0.0):
    rng = np.random.default_rng(seed)
    acc, gyr, tc, stamp, z, td_true = make_measurements(sc, rng)
    f = EkfRio(sc, mode, td_init)
    t_f, n_imu = 0.0, len(sc["t"])
    log = []
    for j in range(len(stamp)):
        tau = tc[j] if mode == "oracle" else stamp[j] + f.td
        tau = min(max(tau, t_f), (n_imu - 1) * DT)
        while t_f < tau - 1e-12:                              # propagate with the IMU to t + t_d
            k = int(t_f / DT + 1e-9)
            t_next = min((k + 1) * DT, tau)
            f.propagate(acc[k], gyr[k], t_next - t_f)
            t_f = t_next
        k = min(int(t_f / DT + 1e-9), n_imu - 1)
        nis, innov = f.update(z[j], acc[k], gyr[k])
        vb = rot(f.th).T @ f.v
        td_log = td_true[j] if mode == "oracle" else f.td
        log.append((t_f, f.th, *vb, *f.p, td_log, np.sqrt(max(f.P[6, 6], 0)), td_true[j], nis, innov))
    L = np.array(log)
    tt = L[:, 0]
    true = {k: np.interp(tt, sc["t"], sc[k]) for k in ["u", "th", "dist"]}
    p_true = np.stack([np.interp(tt, sc["t"], sc["p"][:, i]) for i in range(2)], 1)
    return dict(t=tt, td=L[:, 6], td_sig=L[:, 7], td_true=L[:, 8], e_td=L[:, 6] - L[:, 8],
                ego_err=np.hypot(L[:, 2] - true["u"], L[:, 3]),
                nis=L[:, 9], innov=L[:, 10],
                th=L[:, 1], p=L[:, 4:6], th_true=true["th"], p_true=p_true, dist=true["dist"])


# ----------------------------------------------------------------- metrics
def rmse(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else np.nan


def conv_time(res, t0=0.0, tol=CONV_TOL):
    """Time after t0 from which |e_td| stays below tol until the end (inf if never)."""
    m = res["t"] >= t0
    t, e = res["t"][m], np.abs(res["e_td"][m])
    bad = np.nonzero(e >= tol)[0]
    if len(bad) == 0:
        return 0.0
    if bad[-1] == len(e) - 1:
        return np.inf
    return float(t[bad[-1] + 1] - t0)


def reach_time(res, t0=0.0, tol=CONV_TOL):
    """First time after t0 at which |e_td| < tol (inf if never)."""
    m = res["t"] >= t0
    ok = np.nonzero(np.abs(res["e_td"][m]) < tol)[0]
    return float(res["t"][m][ok[0]] - t0) if len(ok) else np.inf


def rpe(res, mask=None):
    """Relative translation error over RPE_DIST metres, expressed in the start frame [m]."""
    d, th, p, tht, pt = res["dist"], res["th"], res["p"], res["th_true"], res["p_true"]
    j = np.searchsorted(d, d + RPE_DIST)
    i = np.nonzero((j < len(d)) & (mask if mask is not None else True))[0]
    errs = []
    for a, b in zip(i, j[i]):
        e = rot(th[a]).T @ (p[b] - p[a]) - rot(tht[a]).T @ (pt[b] - pt[a])
        errs.append(np.hypot(*e))
    return rmse(np.array(errs))


def ape_aligned(res):
    """Position RMSE after a least-squares SE(2) alignment (as evo does)."""
    P, Q = res["p"], res["p_true"]
    mp, mq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - mp).T @ (Q - mq))
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return rmse(np.linalg.norm((P - mp) @ R.T + mq - Q, axis=1))


def summarize(res, mask=None):
    m = np.ones_like(res["t"], bool) if mask is None else mask
    return dict(ego=rmse(res["ego_err"][m]), rpe=rpe(res, m), ape=ape_aligned(res),
                innov=rmse(res["innov"][m]), gate_fail=float(np.mean(res["nis"][m] > CHI2_99)),
                nis=float(np.mean(res["nis"][m])))


def avg(runs, key, mask_fn=None):
    return float(np.mean([summarize(r, mask_fn(r) if mask_fn else None)[key] for r in runs]))


def fmt_t(x):
    return "never" if not np.isfinite(x) else f"{x:.1f}"


def ms(x):
    return 1000 * np.asarray(x)


# ----------------------------------------------------------------- experiments
def e1_injected_delay(seeds):
    rows = []
    for td in [-0.200, -0.150, -0.113, -0.050, 0.0, 0.050]:
        sc = make_scenario("city", 60.0, lambda _t, td=td: td)
        out = {m: [run(sc, m, s) for s in seeds] for m in ["no-TC", "TC", "oracle"]}
        e_fin = [abs(np.mean(r["e_td"][r["t"] > 50])) for r in out["TC"]]
        row = dict(td=td, e_td=np.mean(e_fin), e_td_sd=np.std(e_fin),
                   conv=np.median([conv_time(r) for r in out["TC"]]))
        for m, rs in out.items():
            for k in ["ego", "rpe", "ape", "gate_fail", "innov"]:
                row[f"{k}_{m}"] = avg(rs, k)
        rows.append(row)
    return rows


def e2_init(seed):
    sc = make_scenario("city", 60.0, lambda _t: TD_PAPER)
    return {ti: run(sc, "TC", seed, td_init=ti) for ti in [0.0, -0.25, 0.10]}


def e3_excitation(seeds):
    rows = []
    for A in [0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0]:
        sc = make_scenario("excite", 60.0, lambda _t: TD_PAPER, A=A)
        tc_runs = [run(sc, "TC", s) for s in seeds]
        no_runs = [run(sc, "no-TC", s) for s in seeds]
        convs = [conv_time(r) for r in tc_runs]
        rows.append(dict(
            A=A, conv=np.median(convs), n_conv=int(sum(np.isfinite(convs))),
            e_td=np.median([abs(np.mean(r["e_td"][r["t"] > 50])) for r in tc_runs]),
            sig_td=np.median([r["td_sig"][-1] for r in tc_runs]),
            ego_no=avg(no_runs, "ego"), ego_tc=avg(tc_runs, "ego"),
            gf_no=avg(no_runs, "gate_fail"), gf_tc=avg(tc_runs, "gate_fail")))
    return rows


T_STEP, TD_AFTER, POST = 45.0, TD_PAPER - 0.050, 20.0


def e4_step(seeds):
    sc = make_scenario("city", 90.0, lambda t: TD_PAPER if t < T_STEP else TD_AFTER)
    methods = ["no-TC", "TC", "TC-bigQ", "oracle"]
    out = {m: [run(sc, m, s) for s in seeds] for m in methods}
    pre = lambda r: (r["t"] >= 20) & (r["t"] < T_STEP)
    post = lambda r: (r["t"] >= T_STEP) & (r["t"] < T_STEP + POST)
    rows = []
    for m in methods:
        rs, est = out[m], m.startswith("TC")
        convs = [reach_time(r, T_STEP) for r in rs]
        rows.append(dict(
            m=m,
            jitter=np.mean([np.std(r["e_td"][pre(r)]) for r in rs]) if est else np.nan,
            conv=np.median(convs) if est else np.nan, n_conv=int(sum(np.isfinite(convs))),
            e_td_post=np.mean([rmse(r["e_td"][post(r)]) for r in rs]),
            cons=np.mean([np.mean(np.abs(r["e_td"][post(r)]) <= 3 * r["td_sig"][post(r)] + 1e-9)
                          for r in rs]) if est else np.nan,
            ego_pre=avg(rs, "ego", pre), ego_post=avg(rs, "ego", post),
            rpe_post=avg(rs, "rpe", post), gf_post=avg(rs, "gate_fail", post),
            nis_post=avg(rs, "nis", post)))
    return rows, out


T_BRAKE = 40.0
BRAKE_WIN = (T_BRAKE, T_BRAKE + 3.3)                 # braking from 25 to 5 m/s


def e5_cruise_brake(seeds):
    """Low excitation for 40 s (highway cruise), then an AEB-like braking event."""
    sc = make_scenario("aeb", 50.0, lambda _t: TD_PAPER)
    cases = [("no-TC", "no-TC", 0.0), ("TC cold start", "TC", 0.0), ("TC warm start", "TC", TD_PAPER),
             ("oracle", "oracle", 0.0)]
    out = {name: [run(sc, m, s, td_init=ti) for s in seeds] for name, m, ti in cases}
    brake = lambda r: (r["t"] >= BRAKE_WIN[0]) & (r["t"] < BRAKE_WIN[1])
    rows = []
    for name, m, _ in cases:
        rs, est = out[name], m.startswith("TC")
        i0 = [np.searchsorted(r["t"], T_BRAKE) for r in rs]
        rows.append(dict(
            name=name,
            e_on=np.mean([abs(r["e_td"][i]) for r, i in zip(rs, i0)]),
            sig_on=np.mean([r["td_sig"][i] for r, i in zip(rs, i0)]) if est else np.nan,
            ego_cruise=avg(rs, "ego", lambda r: (r["t"] > 5) & (r["t"] < T_BRAKE)),
            ego_brake=avg(rs, "ego", brake),
            peak=np.mean([np.max(r["ego_err"][brake(r)]) for r in rs]),
            gf_brake=avg(rs, "gate_fail", brake)))
    return rows, out


def e6_ablation(seeds):
    """Which IMU error makes t_d drift during the cruise of E5? Overrides module constants per row."""
    g = globals()
    keep = {k: g[k] for k in ["SIG_ACC", "SIG_GYR", "LEVER", "CROSS", "BIAS_ACC"]}
    cfgs = [("default", {}),
            ("gyro noise = 0", dict(SIG_GYR=0.0)),
            ("lever arm = 0", dict(LEVER=np.zeros(2))),
            ("accel noise = 0", dict(SIG_ACC=0.0)),
            ("all IMU noise = 0 (biases kept)", dict(SIG_ACC=0.0, SIG_GYR=0.0))]
    rows = []
    for label, over in cfgs:
        g.update(keep)
        g.update(over)
        g["CROSS"] = np.array([-g["LEVER"][1], g["LEVER"][0]])
        sc = make_scenario("aeb", 50.0, lambda _t: TD_PAPER)
        rs = [run(sc, "TC", s) for s in seeds]
        i0 = [np.searchsorted(r["t"], T_BRAKE) for r in rs]
        e = np.array([r["e_td"][i] for r, i in zip(rs, i0)])
        sg = np.array([r["td_sig"][i] for r, i in zip(rs, i0)])
        rows.append(dict(label=label, mean=e.mean(), spread=e.std(), sig=sg.mean(),
                         ratio=np.mean(np.abs(e) / sg),
                         ego_brake=avg(rs, "ego", lambda r: (r["t"] >= BRAKE_WIN[0]) & (r["t"] < BRAKE_WIN[1]))))
    g.update(keep)
    return rows


# ----------------------------------------------------------------- plots
COLORS = {"no-TC": "tab:red", "TC": "tab:blue", "TC-bigQ": "tab:orange", "oracle": "k"}


def plot_e1_e2(e1, e2):
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    for ti, r in e2.items():
        ax[0].plot(r["t"], ms(r["td"]), label=f"init {ms(ti):+.0f} ms")
    ax[0].axhline(ms(TD_PAPER), color="k", ls="--", lw=1, label="true -113 ms")
    ax[0].set(xlabel="time [s]", ylabel="t_d estimate [ms]", title="E2: TC converges from any init", xlim=(0, 20))
    ax[0].legend(fontsize=8)
    td = ms([r["td"] for r in e1])
    ax[1].errorbar(td, ms([r["e_td"] for r in e1]), yerr=ms([r["e_td_sd"] for r in e1]), fmt="o-", capsize=3)
    ax[1].set(xlabel="injected t_d [ms]", ylabel="|t_d error| at 50-60 s [ms]", title="E1: offset error vs injected delay")
    for m in ["no-TC", "TC", "oracle"]:
        ax[2].plot(td, [r[f"ego_{m}"] for r in e1], "o-", color=COLORS[m], label=m)
    ax[2].set(xlabel="injected t_d [ms]", ylabel="ego-velocity RMSE [m/s]", title="E1: trajectory residual")
    ax[2].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig1_e1_e2_offset_error.png"), dpi=120)
    plt.close(fig)


def plot_e3(e3):
    A = [r["A"] for r in e3]
    fig, ax = plt.subplots(1, 2, figsize=(10.5, 4.0))
    conv = [r["conv"] if np.isfinite(r["conv"]) else 60 for r in e3]
    ax[0].plot(A, conv, "o-")
    ax[0].axhline(60, color="gray", ls=":", lw=1)
    ax[0].text(A[-1], 56, "60 s = never", ha="right", fontsize=8, color="gray")
    ax[0].set(xlabel="acceleration amplitude [m/s^2]", ylabel="convergence time [s]",
              title="E3: t_d needs excitation to converge")
    ax[1].plot(A, [r["ego_no"] for r in e3], "o-", color=COLORS["no-TC"], label="no-TC")
    ax[1].plot(A, [r["ego_tc"] for r in e3], "o-", color=COLORS["TC"], label="TC")
    ax[1].set(xlabel="acceleration amplitude [m/s^2]", ylabel="ego-velocity RMSE [m/s]",
              title="E3: a wrong t_d only hurts when there is motion")
    ax[1].legend(fontsize=8)
    for a in ax:
        a.set_xscale("symlog", linthresh=0.1)
        a.set_xticks(A)
        a.set_xticklabels([f"{x:g}" for x in A])
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig2_e3_excitation.png"), dpi=120)
    plt.close(fig)


def plot_e4(out):
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    r0 = out["TC"][0]
    ax[0].plot(r0["t"], ms(r0["td_true"]), "k--", lw=1.2, label="true t_d")
    for m in ["TC", "TC-bigQ"]:
        r = out[m][0]
        ax[0].plot(r["t"], ms(r["td"]), color=COLORS[m], lw=1.2, label=m)
    ax[0].fill_between(r0["t"], ms(r0["td"] - 3 * r0["td_sig"]), ms(r0["td"] + 3 * r0["td_sig"]),
                       color="tab:blue", alpha=0.15, label="TC +-3 sigma")
    ax[0].set(ylabel="t_d [ms]", ylim=(-200, -80), title="E4: radar delay grows by 50 ms at t = 45 s")
    ax[0].legend(fontsize=8, ncol=4, loc="lower left")
    k = 15
    for m in ["no-TC", "TC", "TC-bigQ", "oracle"]:
        r = out[m][0]
        ax[1].plot(r["t"], np.convolve(r["ego_err"], np.ones(k) / k, mode="same"), color=COLORS[m], lw=1.1, label=m)
    ax[1].set(xlabel="time [s]", ylabel="ego-velocity error [m/s] (1 s mean)", xlim=(0, 90))
    ax[1].legend(fontsize=8, ncol=4)
    for a in ax:
        a.grid(alpha=0.3)
        a.axvline(T_STEP, color="gray", lw=0.8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig3_e4_step_failure.png"), dpi=120)
    plt.close(fig)


def plot_e5(out):
    colors = {"no-TC": "tab:red", "TC cold start": "tab:blue", "TC warm start": "tab:purple", "oracle": "k"}
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for name in ["TC cold start", "TC warm start"]:
        r = out[name][1]
        ax[0].plot(r["t"], ms(r["e_td"]), color=colors[name], lw=1.2, label=f"{name}: t_d error")
        ax[0].fill_between(r["t"], ms(r["e_td"] - 3 * r["td_sig"]), ms(r["e_td"] + 3 * r["td_sig"]),
                           color=colors[name], alpha=0.12)
    ax[0].axhline(0, color="k", lw=0.8)
    ax[0].set(ylabel="t_d error [ms] (band: +-3 sigma)", ylim=(-150, 400),
              title="E5 failure case: t_d drifts during a 40 s cruise, then braking starts at t = 40 s")
    ax[0].legend(fontsize=8, loc="upper left")
    for name in colors:
        r = out[name][1]
        ax[1].plot(r["t"], r["ego_err"], color=colors[name], lw=1.0, label=name)
    ax[1].set(xlabel="time [s]", ylabel="ego-velocity error [m/s]", xlim=(0, 50), ylim=(0, 2.5))
    ax[1].legend(fontsize=8, ncol=4, loc="upper left")
    for a in ax:
        a.grid(alpha=0.3)
        a.axvspan(*BRAKE_WIN, color="tab:red", alpha=0.07)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig4_e5_cruise_brake_failure.png"), dpi=120)
    plt.close(fig)


# ----------------------------------------------------------------- main
def main():
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    seeds = list(range(10))
    L = ["# Results (auto-generated by rio_tc_sim.py, mean of 10 seeds unless stated)", ""]

    e1 = e1_injected_delay(seeds)
    L += ["## E1 - injected delay, city motion, 60 s", "",
          "| injected t_d [ms] | TC \\|t_d err\\| [ms] | TC conv. [s] | gate-fail no-TC / TC / oracle "
          "| ego-vel RMSE no-TC / TC / oracle [m/s] | RPE 10 m no-TC / TC / oracle [m] | APE no-TC / TC / oracle [m] |",
          "|---|---|---|---|---|---|---|"]
    for r in e1:
        tri = lambda k, f: " / ".join(f.format(r[f"{k}_{m}"]) for m in ["no-TC", "TC", "oracle"])
        L.append(f"| {ms(r['td']):+.0f} | {ms(r['e_td']):.2f} +- {ms(r['e_td_sd']):.2f} | {fmt_t(r['conv'])} "
                 f"| {tri('gate_fail', '{:.1%}')} | {tri('ego', '{:.3f}')} | {tri('rpe', '{:.3f}')} | {tri('ape', '{:.2f}')} |")
    print("E1 done", round(time.time() - t0), "s")

    e2 = e2_init(0)
    L += ["", "## E2 - convergence from different initial t_d (true -113 ms, seed 0)", "",
          "| init [ms] | t_d after 60 s [ms] | conv. time [s] |", "|---|---|---|"]
    for ti, r in e2.items():
        L.append(f"| {ms(ti):+.0f} | {ms(r['td'][-1]):.1f} | {fmt_t(conv_time(r))} |")

    e3 = e3_excitation(seeds)
    L += ["", "## E3 - excitation sweep (benchmark extension), t_d = -113 ms, 60 s", "",
          "| accel amplitude [m/s^2] | TC conv. [s] (median) | runs converged | TC \\|t_d err\\| 50-60 s [ms] "
          "| final sigma_td [ms] | ego-vel RMSE no-TC / TC [m/s] | gate-fail no-TC / TC |",
          "|---|---|---|---|---|---|---|"]
    for r in e3:
        L.append(f"| {r['A']:.2f} | {fmt_t(r['conv'])} | {r['n_conv']}/{len(seeds)} | {ms(r['e_td']):.1f} "
                 f"| {ms(r['sig_td']):.1f} | {r['ego_no']:.3f} / {r['ego_tc']:.3f} | {r['gf_no']:.1%} / {r['gf_tc']:.1%} |")
    print("E3 done", round(time.time() - t0), "s")

    e4, out4 = e4_step(seeds)
    L += ["", f"## E4 - limitation: t_d steps -113 -> -163 ms at t = {T_STEP:.0f} s (city, 90 s)", "",
          "| method | t_d jitter 20-45 s [ms] | time to reach \|err\| < 10 ms after step [s] | runs reached "
          "| t_d RMSE 45-65 s [ms] | 3-sigma consistency 45-65 s | ego-vel RMSE 20-45 / 45-65 s [m/s] "
          "| RPE 10 m 45-65 s [m] | gate-fail 45-65 s | mean NIS 45-65 s (ideal 2) |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in e4:
        est = r["m"].startswith("TC")
        L.append((f"| {r['m']} | {ms(r['jitter']):.2f} | {fmt_t(r['conv']) if est else '-'} "
                  f"| {str(r['n_conv']) + '/' + str(len(seeds)) if est else '-'} | {ms(r['e_td_post']):.1f} "
                  f"| {'-' if np.isnan(r['cons']) else format(r['cons'], '.0%')} | {r['ego_pre']:.3f} / {r['ego_post']:.3f} | {r['rpe_post']:.3f} "
                  f"| {r['gf_post']:.1%} | {r['nis_post']:.1f} |").replace("nan", "-"))
    print("E4 done", round(time.time() - t0), "s")

    e5, out5 = e5_cruise_brake(seeds)
    L += ["", f"## E5 - failure case: 40 s cruise at 25 m/s (no excitation), then -6 m/s^2 braking (t_d = -113 ms)", "",
          "| method | \|t_d err\| at brake onset [ms] | sigma_td at onset [ms] | ego-vel RMSE cruise 5-40 s [m/s] "
          "| ego-vel RMSE braking [m/s] | peak ego-vel error braking [m/s] | gate-fail braking |",
          "|---|---|---|---|---|---|---|"]
    for r in e5:
        L.append((f"| {r['name']} | {ms(r['e_on']):.0f} | {ms(r['sig_on']):.1f} | {r['ego_cruise']:.3f} "
                  f"| {r['ego_brake']:.3f} | {r['peak']:.2f} | {r['gf_brake']:.0%} |").replace("nan", "-"))
    print("E5 done", round(time.time() - t0), "s")

    e6 = e6_ablation(list(range(20)))
    L += ["", "## E6 - ablation of the E5 drift (TC, t_d error at brake onset, 20 seeds)", "",
          "| configuration | mean t_d error [ms] | spread (std) [ms] | filter sigma_td [ms] | mean \|error\| / sigma "
          "| ego-vel RMSE braking [m/s] |", "|---|---|---|---|---|---|"]
    for r in e6:
        L.append(f"| {r['label']} | {ms(r['mean']):+.0f} | {ms(r['spread']):.0f} | {ms(r['sig']):.1f} "
                 f"| {r['ratio']:.1f} | {r['ego_brake']:.3f} |")
    print("E6 done", round(time.time() - t0), "s")

    plot_e1_e2(e1, e2)
    plot_e3(e3)
    plot_e4(out4)
    plot_e5(out5)
    with open(os.path.join(OUT, "results.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L))
    print(f"\nTotal {time.time() - t0:.0f} s; figures and results.md in {OUT}")


if __name__ == "__main__":
    main()
