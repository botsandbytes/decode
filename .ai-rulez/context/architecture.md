---
priority: high
---

# Project Architecture (FTC decode)

This project is a First Tech Challenge (FTC) robot controller application written in Java.

## Core Architecture Blocks

1. **Subsystem Decomposition**:
   - Robot functionality is split into modular subsystems located under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/`.
   - Key subsystems:
     - `Intake.java`: Front and middle roller control.
     - `Shooter.java`: Double flywheel and hood positioning. Requires `shooter.max_rpm` from config. Exposes only `setTargetPower(double)`; the flywheel is driven exclusively from `periodic()`.
     - `Turret.java`: CRServo-driven turret rotation with dedicated IMU yaw feedback. Aiming is driven by an `AimMode` (`IDLE` / `HOLD` / `AIM_AT_GOAL`) set via `setAimMode()`/`setHoldAngle()`, resolved once per loop in `periodic()`.
     - `ShotController.java`: Owns the shot lifecycle (`startShot`/`stopShot`/`periodic()`) — gates the intake feed on flywheel velocity and (optionally) turret alignment, and drives the Turret's `AimMode` transitions.
     - `Lumos.java`: goBILDA RGB indicator light via `ServoImplEx` with PWM range control. Colors are a `Lumos.Color` enum, not one method per color.
   - Subsystem behaviors are orchestrated using the `com.pedropathing.ivy.Command` command framework.

2. **Robot Container (`Robot.java`)**:
   - `Robot` is the single composition root for a match: it builds the cached `Follower`, all subsystems (`Intake`, `Shooter`, `Turret`), `Sentinel`, `Casablanca`, `ShotController`, and `VisionUtil`, and is constructed once per OpMode from a `MatchProfile` (see block 6).
   - `Robot.update()` is called once per loop and is the *only* place subsystem state advances: it clears the bulk cache, updates the `Follower`, then calls `periodic()` on `Intake`, `Shooter`, `Turret`, and `ShotController` in that order, then runs `Scheduler.execute()`.
   - Commands and controllers only set target state (e.g. `shooter.setTargetPower(x)`, `turret.setAimMode(...)`); they never write to hardware directly. This "single writer per actuator" contract is what `periodic()` exists to enforce — see the code-quality rule of the same name.

3. **Drivetrain & Pathing (Pedro Pathing)**:
   - Drivetrain control, localizer, and path-following are managed by **Pedro Pathing**.
   - Custom constants (mass, PIDF, predictive braking) are configured in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`.
   - `Constants.createCachedFollower(hardwareMap)` is the standard entry point: it wraps all four drivetrain motors in `CachingDcMotorEx`, re-registers them into `hardwareMap`, and builds the `Follower` in one call. Only calibration/tuning OpModes that intentionally bypass caching should call the plain `Constants.createFollower(hardwareMap)` directly.

4. **Hardware Abstraction & Performance (dev.frozenmilk Dairy)**:
   - Utilizes the **Dairy** framework (Core, Pasteurized, CachingHardware, Sloth) for optimization.
   - Drivetrain motors, shooter flywheels, and intake rollers use `dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx` to cache write commands, significantly reducing redundant I2C writes and loop times.

5. **Collision Avoidance (Casablanca & Sentinel)**:
   - `Sentinel` and `Casablanca` are per-match **instances** owned by `Robot`, not static utility classes: `new Sentinel(alliance)` computes the goal zones and launch zones once, and `new Casablanca(sentinel)` reads its friction/smoothing/protection config from it. There is no static "current alliance" global — alliance flows in through the constructor via `MatchProfile`.
   - `Sentinel` checks robot footprint geometry against protected alliance goal zones. It first tries `android.graphics.Path.op(INTERSECT)`, and falls back to a pure-Java Separating Axis Theorem (SAT) polygon-intersection check when `Path.op` isn't available (Robolectric/JVM unit tests) — this is what makes the footprint-rotation and intersection math unit-testable per the code-quality coverage rule, without weakening the production (on-device) path.
   - `Casablanca` dynamically reduces velocity or repels the robot to prevent violating opponent goals, using a `PredictiveBrakingController` to gauge required stopping distance. Depth (X-axis) and side (Y-axis) protection are computed by the same `calculateAxisState` helper called twice with the appropriately-swapped bounds — keep the axis pairing symmetric when touching this code (see `MathSafetyTest#testCasablancaSymmetryAndCollisionMath`).

6. **Alliance-Parameterized OpModes**:
   - There is one `TeleOp.java`, not per-alliance TeleOp files. It reads `Alliance` from the blackboard (falling back to `RED`), builds a `MatchProfile` via `config.loadMatchProfile(alliance)`, and constructs `Robot` from it.
   - Autonomous logic lives in two alliance-agnostic abstract classes, `AllianceAutoNew` and `AllianceOppositeNew`, each taking an `Alliance` in its constructor and loading its poses via `config` (see block 8).
   - `UnifiedAutos.java` registers the four concrete OpModes (`Blue/Red` × `AutoNew/OppositeNew`) programmatically via `@OpModeRegistrar`, as thin one-line subclasses — there are no more hand-duplicated per-alliance auto files.
   - Each auto/teleop OpMode persists its final pose to the blackboard (`"RED_POSE"` / `"BLUE_POSE"`) so the next OpMode (typically TeleOp) can resume from the correct field position.

7. **Dynamic Configuration System**:
   - Config lives in one YAML file, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml`, with human-readable descriptions and min/max constraints kept separately in the sibling `config-docs.yaml` (keyed by leaf name).
   - Runtime loading is handled by the `config-compiler` Gradle module (`org.firstinspires.ftc.teamcode.config.ConfigLoader`), a small reflection-based YAML→POJO binder. It prefers an ADB-pushed override at `/sdcard/FIRST/teamcode/config.yaml` over the bundled classpath resource, so config can be hot-reloaded without an APK rebuild (`./gradlew pushConfig` / `./gradlew resetConfig`, backed by `scripts/push_config.py`).
   - `config.java` (under `robot/config/`) is a **generated** typed facade over that loader — nested static classes mirroring the YAML structure — and a `loadMatchProfile(Alliance)` helper. It is regenerated from `config.yaml` + `config-docs.yaml` by `python3 scripts/generate_config.py`, which also regenerates `config-schema.json`; never hand-edit `config.java` or `config-schema.json`.
   - `scripts/check_config_keys.py` (wired into the `checkConfigKeys` Gradle task, part of `verifyBuild`) statically checks `config.yaml` for alliance-name asymmetry (a `red_*` key with no `blue_*` mirror) and fuzzy near-duplicate keys, failing the build on the former.
   - The old `utilities/ConfigLoader.java` (string-path `getDouble`/`getInt`/... lookups) is `@Deprecated` and only kept as a thin shim over the new loader for backward compatibility — new code should read from `config`, not call it directly.

8. **Dedicated Records Package**:
   - The `org.firstinspires.ftc.teamcode.records` package contains simple, immutable record types and structural data elements:
     - `LaunchParameters`: Immutable shooter calculation parameters.
     - `PIDGains`: Holder for kp, ki, and kd gains.
     - `Alliance`: RED / BLUE enum.
     - `Field`: Dynamic goal coordinates derived from active alliance.
     - `MatchProfile`: Bundles `Alliance`, goal X/Y, and the four match poses (start/score/drink/park) for a given alliance; always passed into `Robot`'s constructor. `TeleOp` builds it via `config.loadMatchProfile(Alliance)`; `AllianceAutoNew`/`AllianceOppositeNew` build their own from the merged `auto_poses.*` config since their pose sets differ from TeleOp's. There is no static "current alliance/current match" global anywhere in the codebase — it is always threaded through explicitly via `MatchProfile`.
