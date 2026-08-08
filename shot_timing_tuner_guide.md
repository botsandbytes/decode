# FTC Decode - Shot Timing Tuner OpMode Guide

This guide provides operational instructions, controls mapping, system architecture, telemetry interpretation, and diagnostic workflows for the **Shot Timing Tuner** OpMode (`org.firstinspires.ftc.teamcode.tests.ShotTimingTunerOpMode`).

---

## 1. Overview & Purpose

### Architectural Context: The Single-Shot Pipeline
In the FTC Decode codebase, Autonomous and TeleOp now share **one unified shot implementation**: `ShotController.aimAndShootCommand(follower, sentinel)`. 

Previously, Autonomous executed a separate, degraded shot—using constant flywheel power at every distance, leaving the turret parked at 0°, and feeding on flywheel speed alone. Now, both TeleOp and Autonomous run the full dynamic pipeline:
1. Holds chassis pose (`follower.holdPoint()`).
2. Rotates turret in real-time (`AIM_AT_GOAL`) tracking the target goal azimuth.
3. Positions the hood and calculates distance-based flywheel setpoints via `ShotSolver`.
4. Enforces 3 readiness gates before feeding (`Flywheel Speed Hysteresis`, `Turret Alignment`, `Solver Validity`).
5. Enforces launch zone safety boundary rules (`Sentinel.isLaunchAllowed()`).

In Autonomous, this unified shot is wrapped with a time budget: `race(aimAndShootCommand(follower, sentinel), waitMs(shootWaitMs))`.

### The Problem
Because the shot pipeline now handles turret settling, hood travel, solver calculation, and dynamic flywheel spin-up before firing, each distance has a different execution time. Expiration of an undersized `shoot_wait_ms` window during Autonomous is silent—the auto cycle simply finishes early or fails to feed without throwing a driver station error.

### The Solution
The **Shot Timing Tuner** (`Shot Timing Tuner` under the `Calibration` group) directly benchmarks this unified `aimAndShootCommand` pipeline across a spread of distances.

Instead of guessing timing windows, the OpMode counts balls exiting the flywheel using instantaneous velocity drop detection (`FlywheelDipDetector`). Once the target ball count (default: 3) is reached, the trial completes. The measured duration plus safety margin (`WINDOW_MARGIN_MS = 250 ms`) provides the exact timing window required for `config.auto.shoot_wait_ms`.

---

## 2. Operating Modes: `DRIVE` vs. `SIM`

When launched, the OpMode prompts the driver to select between two execution modes:

| Mode | Physical Chassis Movement | Distance Constraints | Primary Use Case |
| :--- | :--- | :--- | :--- |
| **`DRIVE`** | Robot physically drives along the **Calibration Ray** using Pedro Pathing (`follower.holdPoint()`). | Restricted to the legal field drive box ($\le \sim 122''$). Distances outside the legal boundary are automatically skipped. | Full field validation with active localizer motion, chassis acceleration, and real driving dynamics. |
| **`SIM`** | Chassis remains static (**no wheels turn**). Believed pose is set to the target waypoint (`follower.setPose()`). | No field boundary limits ($\le 136''+$ reachable). | Pit benchmarking, constrained workspace testing, and measuring far distance endpoints without a full field. |

> [!NOTE]
> In **`SIM`** mode, while the drivetrain does not move, **the turret physically slews**, **the hood moves**, and **the shooter fires real balls**. The shooter pipeline behaves identically to live driving because setpoint calculations depend on commanded distance rather than physical chassis movement.

---

## 3. Gamepad Control Mapping (Gamepad 1)

### Phase 1: Pre-Run Mode Selection
| Button / Input | Trigger | Action |
| :--- | :--- | :--- |
| **Right Bumper (RB)** | Press | Toggle between `DRIVE` and `SIM` mode |
| **Button A** | Press | Confirm selected mode & advance |

### Phase 2: DRIVE Waypoint Navigation (DRIVE Mode Only)
| Button / Input | Trigger | Action |
| :--- | :--- | :--- |
| **Button A** | Press | Command robot to drive to next waypoint along calibration ray |
| **Button B** | Press | Skip current distance waypoint |

### Phase 3: Measurement & Shot Tuning (`measureEndpoint`)
| Button / Input | Trigger | Action | Subsystem / System |
| :--- | :--- | :--- | :--- |
| **Right Trigger** | Press (`> 0.3`) | **Fire Test Shot**: Revs flywheel, aims turret, and feeds intake via `aimAndShootCommand`. | `ShotController`, `Shooter`, `Turret` |
| **Left Trigger** | Press (`> 0.3`) | **Master Stop / Abort**: Instantly cuts intake, flywheel power, and shot controller. | All Subsystems |
| **Button X** | Press | **Toggle Intake**: Starts/stops intake rollers at teleop power to pre-load elements. | `Intake` |
| **DPad Up** | Press | **Increase Target Ball Count** (+1 ball, max 9) | Calibration State |
| **DPad Down** | Press | **Decrease Target Ball Count** (-1 ball, min 1) | Calibration State |
| **Right Stick Click (R3)** | Press | **Increase Dip Threshold (+0.5%)**: Decreases dip sensitivity (prevents false double-counts). | `FlywheelDipDetector` |
| **Left Stick Click (L3)** | Press | **Decrease Dip Threshold (-0.5%)**: Increases dip sensitivity (catches shallow dips). | `FlywheelDipDetector` |
| **Button A** | Press | **Accept Measurement**: Record timing results for current distance and save. | Data Persistence |
| **Button B** | Press | **Skip Distance**: Discard current distance measurement and proceed to next endpoint. | State Machine |

---

## 4. Distance Setpoints & Calibration Ray

Trials walk out along the standard **Blue Alliance Calibration Ray** (aimed toward field corner `X=85.0, Y=11.0`):

- **Target Goal**: Blue Goal (`Field.getBlueGoalX()`, `Field.getBlueGoalY()`)
- **Default Test Setpoints**: `52.0"`, `68.0"`, `86.0"`, `104.0"`, `122.0"`, `136.0"` from goal center.
- **Starting Pose**: `(72, 72, 0°)`

---

## 5. Flywheel Dip Detection Physics

Ball counts are derived from energy transfer into game elements:
1. **Velocity Drop**: When a ball enters the flywheel rollers, it absorbs kinetic energy, creating a sharp velocity dip lasting 20–40 ms.
2. **Dynamic Baseline**: `FlywheelDipDetector` calculates dip depth relative to an **Exponential Moving Average (EMA)** running baseline (`alpha = 0.03`), **not** static setpoint RPM.
   - *Why?* During continuous multi-ball bursts, flywheel speed sags below target setpoint. Target-relative thresholds would report a multi-ball burst as one long single dip. Baseline tracking isolates each ball as a distinct event.
3. **Dip Threshold Calibration**:
   - Default Dip Trigger (`dipFraction`): `5.0%` depth below baseline.
   - Recovery Threshold (`recoverFraction`): `2.0%` rise required before next edge.
   - Refractory Lockout (`refractoryMs`): `120 ms` minimum separation between counted balls.
4. **Fine-Tuning Thresholds**:
   - If the telemetry readout shows `deepest dip = 8.5%` and your trigger is `5.0%`, detection is optimal.
   - If the detector misses balls, use **L3** to drop the threshold to `4.5%` or `4.0%`.
   - If noisy flywheel fluctuation triggers phantom balls, use **R3** to raise the threshold to `5.5%` or `6.0%`.

---

## 6. Telemetry Interface & Readouts

During a measurement attempt, Driver Station telemetry displays live pipeline diagnostics:

```text
SIM : 86 in target / 86.0 in now
Load : 3 balls
SHOT : FIRING 840 ms — measuring
Balls this shot : 2 [310, 620]
--- result ---
WINDOW NEEDED : 1070 ms   (config 1500)
First ball : 310 ms   <- cost of arming the shot
Last ball : 820 ms   of 3 balls
A to accept this distance
--- flywheel ---
Flywheel : idling at constant_rpm
Dip now / deepest : 1.2% / 7.8%   (trigger 5.0%)
Detector : watching
Gate : READY (Speed: OK, Aim: OK, Solver: OK)
Aim / solver : ok / valid
Intake : off
```

### Key Indicators:
- **`First ball (ms)`**: Time elapsed from trigger press to first ball launch. Represents total pipeline arming latency (spin-up + turret slew + hood positioning).
- **`WINDOW NEEDED (ms)`**: `Last ball time + 250 ms`. This is the empirical recommendation for `auto.shoot_wait_ms`.
- **`Gate`**: Displays live status of the 3 readiness gates (`Flywheel Velocity`, `Turret Alignment`, `Solver Validity`).

---

## 7. Data Logging & Exported Files

All data is automatically persisted to the Control Hub SD card directory (`/sdcard/FIRST/teamcode/`) after every shot attempt:

### 1. `shot_time_table.yaml`
Human-readable YAML mapping distances to required execution windows:
```yaml
# Generated by Shot Timing Tuner
  shot_time_table:
    # distance_in, shoot_window_ms
    points: [
        52.0, 780,
        68.0, 920,
        86.0, 1070,
        104.0, 1250,
        122.0, 1410,
        136.0, 1580
      ]
```

### 2. `shot_timing_log.csv`
High-frequency per-loop control log for detailed post-run telemetry diagnostics:
- **Columns**: `shot`, `distance_in`, `sim`, `t_ms`, `velocity`, `target_velocity`, `dip_pct`, `baseline`, `ball`, `flywheel_ready`, `aimed`, `solver_valid`, `hood`, `solved_rpm`

---

## 8. Diagnostic Workflow: Troubleshooting Short Shots

If a trial times out (`SHOT_TIMEOUT_MS = 4000 ms`) or returns inconsistent timing:

1. **Check Launch Zone Legality (`Launch Legal`)**:
   - `Sentinel.isLaunchAllowed(pose)` must be true. If the robot pose is outside the legal launch zone, `aimAndShootCommand` will immediately terminate without firing.
2. **Check First Ball Latency (`firstBallMs`)**:
   - If `firstBallMs` > 600 ms, examine `shot_timing_log.csv` to see which gate delayed arming:
     - `flywheel_ready = 0`: Flywheel PID gains require tuning or setpoint jump is too large.
     - `aimed = 0`: Turret heading PID slew rate is too slow or localizer heading error is high.
     - `solver_valid = 0`: Robot pose is outside valid ballistics table boundaries.
3. **Check Ball Dip Depth (`dip_pct`)**:
   - Inspect `Dip now / deepest` on telemetry. If deepest dip is close to the trigger value, adjust using **L3 / R3**.
4. **Verify Physical Loading**:
   - Ensure elements are properly indexed against feed rollers prior to pressing **Right Trigger**.

---
*FTC Decode Team Calibration Documentation.*
