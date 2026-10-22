# Flywheel Shooter: Closed-Loop Velocity Control (`shooter_pid_imp`)

This document describes the shooter flywheel control system introduced on this branch,
the log data that motivated it, and how to verify and fine-tune it on the robot.

- **Mechanism:** 2× Kraken X60, direct-drive, 2× 4" shooter wheels, launching a ½ lb ball.
- **Controller:** onboard TalonFX `VelocityTorqueCurrentFOC` (runs inside the Kraken at its
  full internal rate; the RIO only writes setpoints).
- **Motivating log:** `akit_26-04-10_20-23-35_mnmi2_q41` (189 ball-contact events).

---

## 1. The problem with the previous loop

The flywheel ran `VelocityTorqueCurrentFOC` with `kP = 9.0`, `kS = 1.9`, `kV = 0.042` and
**`kI = 0`**, with output capped at **±10 V**. From the April 10 log:

| Measured quantity | Value |
|---|---|
| Shooting setpoint | 460 rad/s ≈ 73 rotor rev/s (~4,400 rotor RPM) |
| No-ball cruise | 9.04 V, 2.98 A @ 462 rad/s (~95% of 12 V free speed) |
| Velocity dip per ball contact | **19–40% of setpoint** (460 → 270–350 rad/s) |
| Stator current during dips | 40–70 A (vs ~3 A at cruise) |
| Applied voltage during dips | 7–9.6 V (pinning the old 10 V cap) |
| Flywheel inertia (fit from spin-up ramps) | J ≈ 2.1×10⁻⁴ kg·m² (rotor side) |
| Momentum stolen per ball | ≈ 0.1–0.2 N·m·s |
| Baseline during long strings | sags 460 → 400–430 rad/s after ~10 balls |

Why a P-only loop cannot fix this:

1. **Feedback reacts late.** The controller only produces extra torque *after* velocity has
   already dropped. The ball is in the rollers for only ~60–200 ms, so most of the dip
   happens before the loop can respond.
2. **No integrator.** With `kI = 0` there is always a residual error while a load torque is
   present — this is the sustained baseline sag during long shooting strings.
3. **Voltage headroom was clipped.** Recovery torque requires current; current requires
   voltage above back-EMF (~9 V at setpoint). The 10 V cap left almost nothing for
   transients.
4. **The flywheel is light.** With J ≈ 2.1×10⁻⁴ kg·m², one ball's engagement momentum is a
   large fraction of the stored momentum. A short instantaneous dip at engagement is
   **physics (Δω = ΔL_ball / J), not a tuning failure** — control can pre-load the momentum
   and recover the residual, but cannot make the contact instant free.

## 2. The control scheme

Three layers, all executed onboard the motor:

```
out (A) = kS·sign(refω) + kV·refω            ← feedforward: baseline torque
        + kP·(refω − ω) + kI·∫(refω − ω)     ← feedback: disturbance rejection
        + BallLoadAmps  (while kicker running) ← ball-load momentum feedforward
```

### Layer 1 — Ball-load momentum feedforward (the key idea)

Ball arrival is *predictable*: the kicker is commanded on before the ball reaches the
rollers. When `ShooterSubsystem.setKickerRunning(true, false)` is called, the flywheel's
control request now includes:

```java
m_leftMotorSetpoint.withVelocity(velocity).withFeedForward(Amps.of(N));
```

`VelocityTorqueCurrentFOC.FeedForward` is added directly to the closed-loop output, so the
rollers carry N·kT ≈ N·0.0097 N·m of **extra torque per motor at the exact instant of ball
engagement**, instead of waiting for the velocity error to appear. The bias is:

- toggled by kicker state (off when stopped, off during reverse/unclogging);
- live-tunable from SmartDashboard: **`Flywheel/BallLoadAmps`** (default 10 A);
- logged as `/ShooterSubsystem/Flywheel/BallLoadAssistActive`.

### Layer 2 — PI feedback with full voltage authority

- `kI` added (starts 0.5) — removes the per-ball residual and long-string baseline sag.
- Peak voltage raised 10 V → 12 V — the log showed the loop hitting the old cap mid-dip.
- `kP`/`kS`/`kV` unchanged for now; re-tune in that order if needed (see §4).
- Velocity status signals logged at 250 Hz (other signals stay at 50 Hz) so dip shapes are
  visible in AdvantageScope instead of hidden between 50 Hz samples.
- Setpoint logged as `/ShooterSubsystem/Flywheel/VelocitySetpoint` for step-response plots.

> **Units note:** with `VelocityTorqueCurrentFOC` the slot gains are torque-current based:
> `kS` in Amps, `kV` in A/(rotor rot/s), `kA` in A/(rotor rot/s²). Velocity feedback and
> error are in **rotor rotations per second**.

### Layer 3 — Know the physics floor

Instantaneous contact dip ≈ ΔL_ball/J. Control minimizes the sustained deficit and the
recovery time; if the instantaneous dip must shrink further, increase J (heavier wheels /
weighted hubs) — every dip scales as 1/J.

### Code map

| File | Change |
|---|---|
| `subsystems/shooter/ShooterConfiguration.java` | `kI` added; voltage cap 10→12 V; tuning comments |
| `subsystems/shooter/flywheel/io/FlywheelIO.java` | new `setBallLoadAssist(boolean)` interface |
| `subsystems/shooter/flywheel/io/FlywheelIOReal.java` | feedforward injection, `Flywheel/BallLoadAmps` live tuning, 250 Hz velocity logging |
| `subsystems/shooter/flywheel/Flywheel.java` | `setBallLoadAssist` pass-through + logging, setpoint logging |
| `subsystems/shooter/ShooterSubsystem.java` | kicker state gates the ball-load assist |
| `tools/flywheel_dip_analysis.py` | offline dip/statistics analyzer for AdvantageKit CSV exports |

## 3. Verification

1. **Export a log:** run a teleop/auto session with shooting, then export the AdvantageKit
   `.mtlog` to CSV (AdvantageScope → Export → CSV, or `ktuner`/`advantagescope` CLI).
2. **Run the analyzer:**

   ```bash
   python3 tools/flywheel_dip_analysis.py <export.csv>
   ```

   It prints every ball-contact dip event (depth %, duration, recovery ms, current,
   voltage), a plant identification (J, friction), and cruise headroom.

3. **Acceptance targets** (all at your normal `ShootingRPS`):

   | Metric | Baseline (Apr 10 log) | Target |
   |---|---|---|
   | Median dip depth | ~23% | < 8–10% (engagement physics floor) |
   | Dip recovery time | often unresolved during string | < 0.15 s to within 1% |
   | Baseline drift over a 10-ball string | −5 to −12% | ≈ 0% |
   | Voltage at dip minimum | pinned at cap | not pinned at 12 V |

4. **AdvantageScope plot to keep on the tuning laptop:**
   `/ShooterSubsystem/Flywheel/VelocitySetpoint`, `…/LeftMotorVelocity`, `…/RightMotorVelocity`,
   `…/LeftMotorStatorCurrent`, `/ShooterSubsystem/Flywheel/BallLoadAssistActive`.

## 4. Fine-tuning procedure

All knobs are live on SmartDashboard (no redeploy): `Flywheel/KP`, `Flywheel/KI`,
`Flywheel/KS`, `Flywheel/KV`, `Flywheel/BallLoadAmps`, `ShootingRPS`. Change **one value at
a time**, wait ~1 s for it to apply, fire a volley, then look at the log.

**Step 0 — Baseline.** Set `BallLoadAmps = 0`. Log a shooting routine. This reproduces the
"before" behavior for comparison.

**Step 1 — Feedforward (kV, then kS).** Only redo these if you changed wheels/motor config,
or if cruise voltage/current look wrong. With `kP = kI = 0` (use the values noted down
first!), command the shooting setpoint and adjust `KV` until the wheel settles at setpoint
(±1 rotor rps). Then raise `KS` until the startup response stops creeping and settles
cleanly. Restore `kP`/`kI`.

**Step 2 — Ball-load bias.** Raise `Flywheel/BallLoadAmps` in 5 A steps with volleys
between changes. You are moving the *center* of the dips up toward setpoint:
- tune until the **time-averaged velocity while kicking ≈ setpoint** (kills baseline sag);
- stop before velocity overshoots *above* setpoint between balls.
Expected working band from the April data: **10–25 A per motor**.

**Step 3 — Integrator.** Attack dip *shape* with `Flywheel/KI` (start 0.5):
- dip still bottoms out and lingers → raise (0.5 → 1 → 2 → 4);
- overshoot above setpoint right after each ball exits, or hunting between shots → lower.
Phoenix 6 26.1 exposes no explicit anti-windup setting; each `setControl` resend (every RIO
loop while shooting) limits integrator windup, but keep `kI` as low as achieves the target.

**Step 4 — Proportional.** If the bottom of each dip is flat-bottomed (saturated) and
deep, first confirm the voltage is no longer pinned during dips — the 12 V cap should fix
most of this. If recovery is still slow *without* saturation, raise `kP` until the empty-wheel
0→setpoint step rings, then back off 30%.

**Step 5 — Sweep and confirm.** Repeat a quick pass at your lowest and highest `ShootingRPS`
values, then export a full session and re-run the analyzer. Check both wheels dip
symmetrically — a wheel dipping much deeper indicates its own friction or a `kS`/`kV`
mismatch (the configs are separate: `kLeftFlywheelMotorConfiguration` /
`kRightFlywheelMotorConfiguration`).

### Symptom → action quick table

| Symptom in the velocity trace | Likely cause | Action |
|---|---|---|
| All dips centered well below setpoint | ball momentum not pre-loaded | raise `BallLoadAmps` |
| Velocity above setpoint whenever kicker runs | too much bias | lower `BallLoadAmps` |
| Dips recover but slow (>0.25 s) | weak feedback | raise `kI` (then `kP` if unsaturated) |
| Dips flat-bottomed, current ≫ previous, voltage pinned at 12 V | at torque ceiling | hardware: check battery/bus sag, supply limit, or add flywheel mass |
| Baseline sags over a long string | supply-current throttling or intake droop | check `FlywheelBench`/supply limit (see §5) |
| Ring/oscillation between balls | too much `kI`/`kP` | lower by ~30% |

## 5. Hardware flags from the log

- **Supply current limits** (`SupplyCurrentLimit = 60 A`, lower 40 A for 2.5 s): long
  strings show sustained 40–70 A stator draws; the supply limiter plausibly throttled the
  loop mid-string (matches the observed baseline sag). Check your main breaker and consider
  raising these limits if the electrical system allows.
- Several events show **applied voltage at the dip minimum near 5.5–7 V** — consistent with
  bus/battery sag under load. Watch `RobotController/BatteryVoltage` in a full-session log.
- If after tuning the instantaneous dip is still unacceptable, the fix is mechanical:
  increase flywheel inertia (heavier wheels/hubs). Dips scale as 1/J.
