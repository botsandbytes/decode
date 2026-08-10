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
     - `Turret.java`: CRServo-driven turret rotation, positioned via an absolute analog encoder plus the `Follower`'s pose supplier (no separate IMU). Aiming is driven by an `AimMode` (`IDLE` / `HOLD` / `AIM_AT_GOAL`) set via `setAimMode()`/`setHoldAngle()`, resolved once per loop in `periodic()`.
     - `ShotController.java`: Owns the shot lifecycle (`startShot`/`stopShot`/`periodic()`) — gates the intake feed on flywheel velocity and (optionally) turret alignment, and drives the Turret's `AimMode` transitions. Runs its trajectory solve (`ballistics.ShotSolver`) on a dedicated background thread, not the main loop.
     - `Lumos.java`: goBILDA RGB indicator light via `ServoImplEx` with PWM range control. Colors are a `Lumos.Color` enum, not one method per color.
   - Subsystem behaviors are orchestrated using the `com.pedropathing.ivy.Command` command framework.
   - `VisionUtil` (`utilities/`) is not a `periodic()`-driven subsystem — it's polled directly by the OpMode (`TeleOpBase`) for AprilTag-based pose correction (`updateAprilTagPose()`/`isTagFound()`), streaming toggled on/off around when a fix is needed.

2. **Robot Container (`Robot.java`)**:
   - `Robot` is the single composition root for a match: it builds the cached `Follower`, all subsystems (`Intake`, `Shooter`, `Turret`), `Sentinel`, `Casablanca`, `ShotController`, and `VisionUtil`, and is constructed once per OpMode from a `MatchProfile` (see block 7).
   - `Robot.update()` is called once per loop and is the *only* place subsystem state advances: it clears the bulk cache, updates the `Follower`, then calls `periodic()` on `Intake`, `Shooter`, `ShotController`, and `Turret` in that order, then runs `Scheduler.execute()`.
   - Commands and controllers only set target state (e.g. `shooter.setTargetPower(x)`, `turret.setAimMode(...)`); they never write to hardware directly. This "single writer per actuator" contract is what `periodic()` exists to enforce — see the code-quality rule of the same name.
   - `Robot.shutdown()` stops `ShotController`'s background solver thread. It must be called from the owning OpMode's teardown (`stop()`) — a new `Robot` is constructed on every OpMode init, so skipping this leaks one thread per init.

3. **Drivetrain & Pathing (Pedro Pathing)**:
   - Drivetrain control, localizer, and path-following are managed by **Pedro Pathing**.
   - Custom constants (mass, PIDF, predictive braking) are configured in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`.
   - `Constants.createCachedFollower(hardwareMap)` is the standard entry point: it wraps all four drivetrain motors in `CachingDcMotorEx`, re-registers them into `hardwareMap`, and builds the `Follower` in one call. Only calibration/tuning OpModes that intentionally bypass caching should call the plain `Constants.createFollower(hardwareMap)` directly. Bulk read caching (`LynxModule.BulkCachingMode.MANUAL`) is cleared once per loop in `Robot.update()` — no other code should call `clearBulkCache()`.

4. **Ballistics (Shot Solving)**:
   - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ballistics/` holds the trajectory model used to aim and spin up the shooter: `BallisticsModel`, `ShotSolver`, `ShotTable`, `ShotTimeTable`, `FlywheelFeedforwardFit`.
   - `ShotSolver.solve(...)` is a trajectory-optimization search that is too slow to run synchronously in `periodic()` on every input; `ShotController` runs it continuously on a dedicated daemon thread (`solverThread`) instead. `periodic()` only ever does a cheap `volatile` write to publish the latest `ShotInputs` and a `volatile` read to pick up the most recently completed `ShotSolution` — never call `ShotSolver.solve()` directly from `periodic()` or any control-loop path, and any new state shared between `periodic()` and the solver thread must be `volatile` or guarded the same way (see `ShotController`'s `solveLock`/`pendingInputs`).
   - `FlywheelDipDetector` counts balls fired by the flywheel's brief RPM dip on each ball, measured against a running reference of recent velocity — not the commanded setpoint, since the wheel doesn't recover between shots during a burst.

5. **PID Anti-Windup**:
   - Pedro's `PIDFController`/`FilteredPIDFController` integral term has no clamp and no anti-windup; on a flywheel this causes the integral to charge far past useful values during spin-up and then overshoot violently. `Shooter` uses a separate `AntiWindupIntegrator` (`utilities/`) for the I term instead.
   - When `AntiWindupIntegrator` is in use, the corresponding Pedro `PIDFCoefficients` **must** have its `i` term set to zero — otherwise the integral is double-counted. Never "tune" flywheel oscillation by raising Pedro's own I coefficient.

6. **Collision Avoidance (Sentinel & Casablanca)**:
   - `Sentinel` and `Casablanca` (`utilities/`) are per-match **instances** owned by `Robot`, not static utility classes: `new Sentinel(alliance)` computes the goal zones and launch zones once, and `new Casablanca(sentinel)` reads its friction/smoothing/protection config from it. There is no static "current alliance" global — alliance flows in through the constructor via `MatchProfile`.
   - `Sentinel` checks robot footprint geometry against protected alliance goal zones using JTS Topology Suite (`org.locationtech.jts.geom.Polygon.intersects()`). This provides robust, instant 2D polygon collision detection under pure JUnit 4 unit tests without requiring Android Robolectric wrappers.
   - `Casablanca` dynamically reduces velocity or repels the robot to prevent violating opponent goals, using a `PredictiveBrakingController` to gauge required stopping distance. Depth (X-axis) and side (Y-axis) protection are computed by the same `calculateAxisState` helper called twice with the appropriately-swapped bounds — keep the axis pairing symmetric when touching this code (see `MathSafetyTest#testCasablancaSymmetryAndCollisionMath`).

7. **Alliance-Parameterized OpModes**:
   - There is one `TeleOp`/`TeleOpBase` pair, not per-alliance TeleOp files. It reads `Alliance` from the blackboard (falling back to `RED`), builds a `MatchProfile` via `config.loadMatchProfile(alliance)`, and constructs `Robot` from it.
   - Autonomous logic lives in a generic abstract base, `AllianceAutoBase<T>`, extended by two alliance-agnostic subclasses, `AllianceAutoNew` and `AllianceOppositeNew`, each taking an `Alliance` in its constructor and loading its poses via `config` (see block 9).
   - `UnifiedAutos.java` registers the four concrete OpModes (`Blue/Red` × `AutoNew/OppositeNew`) as thin nested subclasses annotated with the FTC SDK's `@Autonomous` — there are no hand-duplicated per-alliance auto files.
   - Each auto/teleop OpMode persists its final pose to the blackboard (`"RED_POSE"` / `"BLUE_POSE"`) so the next OpMode (typically TeleOp) can resume from the correct field position.

8. **Dynamic Configuration System**:
   - Config lives in one YAML file, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml`, with human-readable descriptions and min/max constraints kept separately in the sibling `config-docs.yaml` (keyed by leaf name).
   - Runtime loading is handled by the `config-compiler` Gradle module (`org.firstinspires.ftc.teamcode.config.ConfigLoader`), a small reflection-based YAML→POJO binder. It prefers an ADB-pushed override at `/sdcard/FIRST/teamcode/config.yaml` over the bundled classpath resource, so config can be hot-reloaded without an APK rebuild (`./gradlew pushConfig` / `./gradlew resetConfig`, backed by the pure-Java `ConfigPusherMain` — no Python dependency).
   - `config.java` (under `robot/config/generated/`) is a **generated** typed facade over that loader — nested static classes mirroring the YAML structure — and a `loadMatchProfile(Alliance)` helper. It is regenerated from `config.yaml` + `config-docs.yaml` by the pure Java generator `ConfigGeneratorMain` (`./gradlew generateConfig`), which also regenerates `config-schema.json`; never hand-edit `config.java` or `config-schema.json`.
   - `checkConfigKeys` (wired into the `checkConfigKeys` Gradle task, part of `verifyBuild`) is a pure Java JavaExec task that statically checks `config.yaml` for alliance-name asymmetry (a `red_*` key with no `blue_*` mirror) and fuzzy near-duplicate keys, failing the build on the former.

9. **Dedicated Records Package**:
   - The `org.firstinspires.ftc.teamcode.records` package contains simple, immutable record types and structural data elements:
     - `PIDGains`: Holder for kp, ki, and kd gains.
     - `Alliance`: RED / BLUE enum.
     - `Field`: Dynamic goal coordinates derived from active alliance.
     - `MatchProfile`: Bundles `Alliance`, goal X/Y, and the four match poses (start/score/drink/park) for a given alliance; always passed into `Robot`'s constructor. `TeleOpBase` builds it via `config.loadMatchProfile(Alliance)`; `AllianceAutoNew`/`AllianceOppositeNew` build their own from the merged `auto_poses.*` config since their pose sets differ from TeleOp's. There is no static "current alliance/current match" global anywhere in the codebase — it is always threaded through explicitly via `MatchProfile`.
     - `ShotInputs` / `ShotSolution` / `BallisticsParameters` / `EndgameSpot`: immutable data carried between `ShotController` and the `ballistics` package (block 4).

10. **Test Layout — Two Different "tests"**:
    - `TeamCode/src/test/java/...`: real JUnit 4 unit tests, run by `./gradlew testDebugUnitTest` (part of `verifyBuild`). This is where new pure-math/logic coverage (rule 9 of code-quality) belongs.
    - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/`: on-robot calibration and tuning **OpModes** (e.g. `FrictionCalibrationOpMode`, `ShooterAutotuneOpMode`, `TurretCenterCalibrationOpMode`) — despite the package name, these are not JUnit tests and don't run under `testDebugUnitTest`. Don't confuse a request to "add a test" between the two; ask or infer from context which is meant.
