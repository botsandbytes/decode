# FTC Decode - TeleOp Controls & Keypress Documentation

This document provides a comprehensive mapping, detailed behavioral description, and safety governance audit for every gamepad button and input in the `TeleOp` OpMode (`org.firstinspires.ftc.teamcode.teleop.TeleOp` extending `TeleOpBase`).

---

## 1. Driver Controls (Gamepad 1)

| Button / Input | Input Type | Trigger Condition | Primary Action & Subsystem Target | Detailed Behavior & Systems Architecture |
| :--- | :--- | :--- | :--- | :--- |
| **Left Stick Y** | Analog Axis | Live Value | **Chassis Forward/Backward Drive** (`follower`) | Drives chassis translation along the longitudinal axis. Value is shaped by a cubic response curve (`-driver.left_stick_y^3`) and clamped to `config.teleop.max_speed` (default `1.0`). Passed through `Casablanca.adjustDriveInput()` for friction compensation, dynamic zone speed scaling, and predictive braking (`Sentinel`). Deflecting stick beyond `\|y\| > 0.2` cancels active automated trajectories (`parkCommand`, `scoreCommand`, `drinkCommand`). |
| **Left Stick X** | Analog Axis | Live Value | **Chassis Strafe Left/Right Drive** (`follower`) | Drives chassis translation along the lateral axis. Value is shaped by a cubic response curve (`-driver.left_stick_x^3`) and clamped to `config.teleop.max_speed`. Passed through `Casablanca.adjustDriveInput()`. Deflecting stick beyond `\|x\| > 0.2` cancels active automated trajectories. |
| **Right Stick X** | Analog Axis | Live Value | **Chassis Rotation (Yaw)** (`follower`) | Drives chassis rotational turning. Value is shaped by a cubic response curve (`-driver.right_stick_x^3`) and clamped to `config.teleop.max_speed`. Processed by `Casablanca` heading controller (assists heading retention when stick is released). Deflecting stick beyond `\|r\| > 0.2` cancels active automated trajectories. |
| **DPad Left** | Digital Button | Press (`onTrue`) | **Auto-Drive to Park Pose** (`parkCommand`) | Schedules `parkCommand` which commands `follower.holdPoint(profile.parkPose())` to navigate automatically to the alliance parking location using Pedro Pathing. Loaded from `config.yaml` (`teleop.poses.<alliance>.park`). Red: `(37.5, 32, 270°)`, Blue: `(105.5, 31, 270°)`. Overrides manual drive while active. Cancelled by driver stick movement or DPad Right. |
| **Right Trigger** | Analog Trigger | Press (`> 0.5`) | **Auto-Drive to Scoring Pose** (`scoreCommand`) | Schedules `scoreCommand` which commands `follower.holdPoint(profile.scorePose())` to navigate automatically to the alliance scoring location using Pedro Pathing. Loaded from `config.yaml` (`teleop.poses.<alliance>.score`). Red: `(61, 21, 0°)`, Blue: `(81, 21, 121°)`. Overrides manual drive while active. Cancelled by driver stick movement or DPad Right. |
| **Left Trigger** | Analog Trigger | Press (`> 0.5`) | **Auto-Drive to Drink Pose** (`drinkCommand`) | Schedules `drinkCommand` which commands `follower.holdPoint(profile.drinkPose())` to navigate automatically to the human player/substation location using Pedro Pathing. Loaded from `config.yaml` (`teleop.poses.<alliance>.drink`). Red: `(129, 60.5, 42°)`, Blue: `(12, 60.5, 140°)`. Overrides manual drive while active. Cancelled by driver stick movement or DPad Right. |
| **DPad Right** | Digital Button | Press (`onTrue`) | **Cancel Auto-Drive / Restore Manual** | Explicitly cancels `parkCommand`, `scoreCommand`, and `drinkCommand`, immediately restoring 100% manual chassis control to the driver sticks. The cancel alone is not sufficient: `holdPoint()` leaves the `Follower` with `manualDrive=false` / `holdingPosition=true`, in which state `Follower.update()` discards the vectors written by `setTeleOpDrive()`. `teleOpDriveCommand.setStart()` calls `follower.startTeleopDrive()` when it reclaims the `Follower`, which is what actually releases the hold — the same mechanism restores control on stick deflection, trigger release, and launch-zone exit. |

---

## 2. Operator Controls (Gamepad 2)

| Button / Input | Input Type | Trigger Condition | Primary Action & Subsystem Target | Detailed Behavior & Systems Architecture |
| :--- | :--- | :--- | :--- | :--- |
| **Button A** | Digital Button | Press (`onTrue`) | **Start Intake** (`intakeCommand`) | Schedules `intakeCommand` to run the front/middle intake rollers at `config.teleop.intake_power` (default `1.0`) and transfer motor at `config.teleop.transfer_power` (default `0.1`). Runs continuously until cancelled by Operator B (stop intake) or Operator Left Trigger (master abort). |
| **Button B** | Digital Button | Press (`onTrue`) | **Stop Intake** (`intakeCommand`) | Cancels `intakeCommand`, whose `setEnd` calls `intake.stop()`. Dedicated counterpart to Button A. Previously B was a duplicate of the manual-rev mapping; with rev moved to Button X, B is the only non-abort way to stop the intake. |
| **Button Y** | Digital Button | Press & Hold (`onTrue` / `onFalse`) | **Target Aim Only** (`aimCommand`) | Gated by `sentinel.isLaunchAllowed(follower.getPose())`. While held: sets turret `AimMode` to `AIM_AT_GOAL`, continuously tracking the `ShotSolver` target azimuth. Does not touch the flywheel. On release: resets turret to `HOLD` mode at 0°. Requires `turret` only. |
| **Right Trigger** | Analog Trigger | Press & Hold (`> 0.5`) | **Auto-Aim & Automatic Shoot** (`aimAndShootCommand`) | Gated by `sentinel.isLaunchAllowed(follower.getPose())`. On press: 1) Holds chassis position (`follower.holdPoint()`). 2) Revs flywheel to target speed (`constant_rpm`). 3) Turret enters `AIM_AT_GOAL` mode, continuously tracking `ShotSolver` target azimuth. 4) `ShotController` verifies 3 readiness gates (Flywheel Speed Hysteresis, Turret Goal Alignment, Ballistics Solver Validity). 5) When all 3 gates pass, intake automatically feeds elements into shooter. On release: Cancels command, stopping shot controller, flywheel, intake, and turret tracking. Automatically terminates if robot leaves launch zone. |
| **Button X** | Digital Button | Press & Hold (`onTrue` / `onFalse`) | **Manual Flywheel Rev / Power Up** (`manualRevCommand`) | While held: spins the shooter flywheel to `config.teleop.manual_rev_power` (default `0.6` / 60% power) without aiming or feeding. On release: stops flywheel power. Requires `shooter` only, and is not launch-zone gated (revving is not launching). Because X (`shooter`) and Y (`turret`) require disjoint subsystems, holding **X + Y together** reproduces the old combined aim-and-pre-spin behavior that X alone used to have. |
| **DPad Up** | Digital Button | Press (`onTrue`) | **Manual Shoot Override** (`shootManualCommand`) | Schedules `shootManualCommand`, starting `ShotController` with `startShot(constantPower, checkAlignment=false)`. Revs flywheel to target speed and feeds intake based purely on flywheel readiness (bypasses turret alignment gate). Serves as a manual fallback if turret sensors are unaligned. |
| **DPad Down** | Digital Button | Press & Hold | **AprilTag Vision Relocalization** | On press: Resumes vision camera streaming (`robot.vision.resumeStreaming()`). While held: Processes AprilTag camera frames. If a valid AprilTag is located, updates Pedro Pathing robot pose (`follower.setPose(visionPose)`) and immediately stops streaming. Displays "Searching for Tag..." telemetry while active. On release: Stops vision streaming. |
| **DPad Right** | Digital Button | Press (`onTrue`) | **Co-Pilot Driver Auto-Cancel** | Operator override button to cancel driver automated pose-holding commands (`parkCommand`, `scoreCommand`, `drinkCommand`). |
| **Left Trigger** | Analog Trigger | Press (`> 0.5`) | **Master Abort / Emergency Stop** | Instantly cancels all active operator commands (`aimAndShootCommand`, `aimCommand`, `shootManualCommand`, `manualRevCommand`, `intakeCommand`), resetting flywheel, intake, and turret states to safe idle. `intakeCommand` was previously omitted, so the abort left the intake rollers running. |

---

## 3. Complete Gamepad Input Audit (Unmapped Inputs)

To guarantee complete documentation of every button on standard FTC gamepads (Logitech F310, DualShock 4, Xbox Wireless):

### Driver Gamepad 1
- **Button A**, **Button B**, **Button X**, **Button Y**: *Unmapped*
- **DPad Up**, **DPad Down**: *Unmapped*
- **Left Bumper (LB)**, **Right Bumper (RB)**: *Unmapped*
- **Left Stick Press (L3)**, **Right Stick Press (R3)**: *Unmapped*
- **Back / Select**, **Start / Menu**, **Guide / PS Button**: *Reserved for FTC System & Driver Station OpMode control*

### Operator Gamepad 2
- **DPad Left**: *Unmapped*
- **Left Bumper (LB)**, **Right Bumper (RB)**: *Unmapped*
- **Left Stick X / Y**, **Right Stick X / Y**: *Unmapped*
- **Left Stick Press (L3)**, **Right Stick Press (R3)**: *Unmapped*
- **Back / Select**, **Start / Menu**, **Guide / PS Button**: *Reserved for FTC System & Driver Station OpMode control*

---

## 4. Architectural Deep Dive & Subsystem Logic

### A. Dynamic Chassis Governance (`Casablanca` & `Sentinel`)
- **Control Response**: Translational and rotational inputs use a cubic mapping curve ($f(x) = x^3$) to provide fine-grained micro-adjustment capability near stick center while preserving 100% max velocity at full stick deflection.
- **Dynamic Speed Scaling**: Inputs are processed by `Casablanca.adjustDriveInput()`, which applies empirical friction compensation parameters (`frictionX`, `frictionY`, `frictionRot`) from `config.yaml`.
- **Predictive Braking (`Sentinel`)**: If chassis trajectory threatens to enter protected opponent alliance zones, `Casablanca` dynamically restricts velocity vectors along the target axis.

### B. Alliance Parameterization & Auto Poses
- TeleOp reads the active match alliance (`RED` or `BLUE`) from the internal blackboard (`blackboard.get("ALLIANCE")`).
- Coordinates for `parkPose()`, `scorePose()`, and `drinkPose()` are dynamically derived from `config.loadMatchProfile(alliance)`.
- If an automated trajectory is active, `hasActiveDriveOverride()` returns `true`, suppressing manual drive command scheduling until the trajectory completes or is interrupted by driver stick input or DPad Right.

### C. Triple-Gate Shot Gating (`ShotController`)
When `aimAndShootCommand` is active, the `ShotController.periodic()` loop enforces three distinct readiness gates before activating the intake feed rollers (`intake.run(feed_intake_power, feed_transfer_power)`):
1. **Flywheel Velocity Gate**: Measured velocity (`shooter.getShooterVelocity()`) must fall within target bounds ($Target \times min\_transfer\_threshold$ to $Target \times max\_velocity\_threshold$). Features latching hysteresis ($releaseLow = feed\_release\_threshold$) so speed drops during ring feed do not cause gate chatter.
2. **Turret Alignment Gate**: Turret current heading must align with the `ShotSolver` calculated azimuth within target tolerance (`turret.isAimed()`).
3. **Solver Validity Gate**: The background thread (`ShotSolver`) must return a valid ballistics solution for the current robot position relative to goal coordinates (`Field.getGoalX(alliance)`, `Field.getGoalY(alliance)`).

### D. Safety Field Boundaries (`Sentinel`)
- Both `aimAndShootCommand` (Right Trigger) and `aimCommand` (Button X) check `sentinel.isLaunchAllowed(follower.getPose())` before scheduling.
- If the robot is positioned in a non-launch field sector, aiming and shooting are strictly prohibited.
- If the robot drifts outside the launch zone during an active shot, `aimAndShootCommand.setDone()` triggers automatic termination.

---
*Generated for FTC Decode Project Governance & Documentation.*
