"""Side checks quoted in the report (B1, C1) that rio_tc_sim.py does not print.

  1. The update Jacobian (state columns and the t_d column) against finite differences.
  2. Residual t_d bias vs IMU rate (E1 setup, t_d = -113 ms, 10 seeds, mean signed error over 50-60 s).

Run:  python check_claims.py   (about 1 minute)   -> results/check_claims.txt
"""
import os

import numpy as np

import rio_tc_sim as R


def h(th, bg, v, g_m):
    return R.rot(th).T @ v + (g_m - bg) * R.CROSS


def check_jacobian():
    th, bg, v, ba = 0.3, 0.002, np.array([9.0, 1.2]), np.array([0.05, -0.02])
    a_m, g_m = np.array([1.1, 2.3]), 0.21
    w, a, Rm = g_m - bg, a_m - ba, R.rot(th)
    H = np.zeros((2, 7))                                  # same construction as EkfRio.update
    H[:, 0] = Rm.T @ R.J.T @ v
    H[:, 1] = -R.CROSS
    H[:, 2:4] = Rm.T
    H[:, 6] = H[:, 0] * w + a
    eps = 1e-6
    ex, ey = np.array([eps, 0.0]), np.array([0.0, eps])
    fd = np.stack([(h(th + eps, bg, v, g_m) - h(th - eps, bg, v, g_m)) / (2 * eps),
                   (h(th, bg + eps, v, g_m) - h(th, bg - eps, v, g_m)) / (2 * eps),
                   (h(th, bg, v + ex, g_m) - h(th, bg, v - ex, g_m)) / (2 * eps),
                   (h(th, bg, v + ey, g_m) - h(th, bg, v - ey, g_m)) / (2 * eps)], 1)

    def h_t(dt, n=2001):                                  # h along the motion (w, a constant, as in the paper)
        s = np.linspace(0.0, dt, n)
        acc = np.stack([R.rot(th + w * x) @ a for x in s])
        v_dt = v + np.sum((acc[1:] + acc[:-1]) / 2 * np.diff(s)[:, None], 0)
        return R.rot(th + w * dt).T @ v_dt + w * R.CROSS

    dt = 1e-4
    fd_t = (h_t(dt) - h_t(-dt)) / (2 * dt)
    return np.abs(H[:, :4] - fd).max(), np.abs(H[:, 6] - fd_t).max()


def td_bias(hz, seeds=range(10)):
    R.IMU_HZ, R.DT = hz, 1.0 / hz
    sc = R.make_scenario("city", 60.0, lambda _t: R.TD_PAPER)
    errs = []
    for s in seeds:
        r = R.run(sc, "TC", s)
        errs.append(np.mean(r["e_td"][r["t"] > 50.0]))
    return 1000 * float(np.mean(errs))


def main():
    e_state, e_td = check_jacobian()
    lines = ["# check_claims.py output", "",
             f"max |H - finite diff| (theta, b_g, v_x, v_y columns): {e_state:.1e}",
             f"max |H_td - d h/dt| (t_d column):                    {e_td:.1e}", ""]
    for hz in [200.0, 400.0]:
        lines.append(f"IMU {hz:.0f} Hz: mean signed t_d error 50-60 s (E1, -113 ms, 10 seeds) = {td_bias(hz):+.2f} ms")
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(R.OUT, "check_claims.txt"), "w") as fh:
        fh.write(text + "\n")


if __name__ == "__main__":
    main()
