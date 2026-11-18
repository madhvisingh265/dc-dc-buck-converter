"""
Switching-level simulation of the buck converter (no SPICE needed).

Each switching period is resolved in small time steps:
  switch ON : inductor sees (Vin - I*Rds_on - Vout)
  switch OFF: inductor sees (-Vf - Vout) while current flows through the Schottky diode;
              if the current reaches zero it stays there (discontinuous mode)

Two experiments:
  1. Open-loop start-up at the designed duty cycle -> steady-state ripple vs. the design equations
  2. Closed-loop PID voltage control with soft-start and a load step (0.5 A -> 1.0 A at 6 ms) -> regulation and recovery

Usage:
    python src/simulate.py
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from design import Spec, design

ROOT = Path(__file__).resolve().parents[1]
FIG = ROOT / "reports" / "figures"

L = 120e-6
C = 100e-6
ESR = 0.05
STEPS_PER_PERIOD = 200
SOFT_START = 2e-3


def run(spec: Spec, t_end, duty_fn, load_fn):
    T = 1 / spec.fsw
    dt = T / STEPS_PER_PERIOD
    n = int(t_end / dt)
    t = np.arange(n) * dt
    iL = np.zeros(n); vC = np.zeros(n); vout = np.zeros(n); duty_log = np.zeros(n)
    i, v, d = 0.0, 0.0, 0.0
    for k in range(n):
        phase = (k % STEPS_PER_PERIOD) / STEPS_PER_PERIOD
        if phase == 0:
            d = duty_fn(t[k], v)  # controller updates once per period
        r_load = spec.vout / load_fn(t[k])
        on = phase < d
        if on:
            vl = spec.vin - i * (spec.rds_on + spec.dcr) - v
        else:
            vl = -spec.vf_diode - i * spec.dcr - v if i > 0 else 0.0
        i = max(i + vl / L * dt, 0.0)
        ic = i - v / r_load
        v = v + ic / C * dt
        iL[k], vC[k], vout[k], duty_log[k] = i, v, v + ic * ESR, d
    return t, iL, vout, duty_log


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    s = Spec()
    calc = design(s)

    # ---------- 1. open loop ----------
    D = calc["duty_with_diode"]
    t, iL, vo, _ = run(s, 6e-3, lambda t, v: D, lambda t: s.iout)
    last = t > 5e-3
    ss = {"vout_mean_V": float(vo[last].mean()), "vout_ripple_pp_mV": float(np.ptp(vo[last]) * 1e3),
          "iL_ripple_pp_A": float(np.ptp(iL[last])), "iL_peak_A": float(iL[last].max())}
    ss["design_iL_ripple_A_at_L_chosen"] = (s.vin - s.vout) * D / (s.fsw * L)
    ss["settling_time_ms_2pct"] = float(t[np.where(np.abs(vo - ss["vout_mean_V"]) > 0.02 * ss["vout_mean_V"])[0][-1]] * 1e3)

    fig, ax = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    ax[0].plot(t * 1e3, vo, lw=0.8); ax[0].set_ylabel("Vout (V)"); ax[0].set_title("Open-loop start-up, 12 V → 5 V, 1 A load")
    ax[1].plot(t * 1e3, iL, lw=0.5, c="tab:orange"); ax[1].set_ylabel("Inductor current (A)"); ax[1].set_xlabel("Time (ms)")
    fig.tight_layout(); fig.savefig(FIG / "startup_open_loop.png", dpi=150)

    z = (t > 5.95e-3)
    fig, ax = plt.subplots(2, 1, figsize=(9, 4.5), sharex=True)
    ax[0].plot(t[z] * 1e6, vo[z]); ax[0].set_ylabel("Vout (V)"); ax[0].set_title("Steady state - five switching periods")
    ax[1].plot(t[z] * 1e6, iL[z], c="tab:orange"); ax[1].set_ylabel("iL (A)"); ax[1].set_xlabel("Time (µs)")
    fig.tight_layout(); fig.savefig(FIG / "steady_state_ripple.png", dpi=150)

    # ---------- 2. closed loop PID with load step ----------
    # PID: the derivative term damps the L-C resonance (~1.5 kHz) that a plain PI controller excites
    kp, ki, kd = 0.05, 100.0, 6e-5
    state = {"integ": 0.0, "last_t": 0.0, "last_v": 0.0}

    def pid(t, v):
        dt = max(t - state["last_t"], 1e-9)
        dv = (v - state["last_v"]) / dt
        state["last_t"], state["last_v"] = t, v
        vref = s.vout * min(t / SOFT_START, 1.0)  # soft-start: ramp the reference over 2 ms
        err = vref - v
        state["integ"] = np.clip(state["integ"] + err * dt, -0.01, 0.01)
        return float(np.clip(D + kp * err + ki * state["integ"] - kd * dv, 0.05, 0.9))

    load = lambda t: 0.5 if t < 6e-3 else 1.0
    t2, iL2, vo2, d2 = run(s, 12e-3, pid, load)
    before = (t2 > 5e-3) & (t2 < 6e-3); after = t2 > 11e-3
    step = (t2 >= 6e-3)
    cl = {"startup_overshoot_pct": float((vo2[t2 < 6e-3].max() / s.vout - 1) * 100), "vout_before_step_V": float(vo2[before].mean()), "vout_after_step_V": float(vo2[after].mean()),
          "undershoot_mV": float((vo2[before].mean() - vo2[step].min()) * 1e3)}
    bad = np.where(step & (np.abs(vo2 - 5.0) > 0.05))[0]
    cl["recovery_time_ms_to_within_1pct"] = float((t2[bad[-1]] - 6e-3) * 1e3) if len(bad) else 0.0
    cl["ripple_after_step_pp_mV"] = float(np.ptp(vo2[after]) * 1e3)

    fig, ax = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    ax[0].plot(t2 * 1e3, vo2, lw=0.8); ax[0].axvline(6, ls="--", c="grey"); ax[0].set_ylabel("Vout (V)")
    ax[0].set_title("Closed-loop PID control - load step 0.5 A → 1.0 A at 6 ms")
    ax[1].plot(t2 * 1e3, d2, c="tab:green"); ax[1].set_ylabel("Duty cycle"); ax[1].set_xlabel("Time (ms)")
    fig.tight_layout(); fig.savefig(FIG / "closed_loop_load_step.png", dpi=150)

    out = {"open_loop": ss, "closed_loop": cl}
    (ROOT / "reports" / "simulation.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
