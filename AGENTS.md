<!--
🤖 AI-RULEZ :: GENERATED FILE — DO NOT EDIT
Project: decode
Generated: 2026-07-23 01:24:20
Source: .ai-rulez/config.toml

NEVER edit this file - modify .ai-rulez/ content instead
Use MCP server: npx -y ai-rulez@latest mcp
Regenerate: ai-rulez generate

Docs: https://github.com/Goldziher/ai-rulez
Content-Hash: blake3:76f08ea57da0cb303f0d1c464cad0e8b1f6a9704586515ffacf4375b770ee24c
Source-Hash: blake3:103384c5979634bfc4353e1b9db1d77afb7f544de5eb32c83c64cc353d7bd729
-->

# decode

AI-powered development governance for FTC decode project

## Rules

### code-quality

**Priority:** high

Follow these coding and design standards in the `decode` codebase:

## Coding Conventions
1. **Dynamic Configuration Over Hardcoding**:
   - Never hardcode mechanical coefficients, tolerances, PID constants, safety margins, velocity scales, or autonomous timing values in Java code.
   - Add or change values in `config.yaml`; access them in Java via the generated `config` facade (e.g. `config.turret.pidf.p`) — do not call the low-level `config.ConfigLoader.load(...)` directly from subsystem code, and never call the deprecated `utilities.ConfigLoader`.
   - When adding a new config key, also add its description (and `min`/`max` where applicable) to `config-docs.yaml`, then run `./gradlew generateConfig` (or `./gradlew verifyBuild`) to regenerate `config.java` and `config-schema.json`. Never hand-edit either generated file.
   - If a new key mentions an alliance (`red_*`/`blue_*`), add its mirror for the other alliance or the `checkConfigKeys` Gradle task (part of `verifyBuild`) will fail the build.

2. **Prefer Libraries Over Custom Implementations**:
   - Never write custom code for something a library can already do — even if that library is not yet installed. Identify and add the appropriate dependency first.
   - Use JTS Topology Suite (`org.locationtech.jts.geom.Coordinate`, `Envelope`, `Polygon`) for 2D spatial geometry and polygon intersections instead of custom record types.
   - Use Pedro Pathing's `com.pedropathing.control.PIDFController` and `PIDFCoefficients` instead of any custom PID/PIDF implementations.
   - Use FTC SDK's `org.firstinspires.ftc.robotcore.external.navigation.AngleUnit` normalization methods instead of custom modulo math.
   - This rule applies to: PID controllers, geometry types, angle math, data structures, interpolation, collections, and anything else with a well-supported library equivalent.
   - Exception: `Sentinel` uses JTS (`Polygon.intersects()`) for robust 2D polygon intersection math, allowing all unit tests to execute under pure JUnit 4 in < 0.1s.

3. **Hardware Optimization via Dairy Caching**:
   - Drivetrain wheels, shooters, and rollers must use caching hardware wrappers (`CachingDcMotorEx`, `CachingServo`, `CachingCRServo`) to optimize loop times.
   - For the drivetrain, call `Constants.createCachedFollower(hardwareMap)` — it wraps all four motors in `CachingDcMotorEx`, registers them back into `hardwareMap`, and builds the `Follower` in one step. Only bypass it (calling `Constants.createFollower` directly) in calibration/tuning OpModes that intentionally want uncached motors.
   - Set sensible tolerances (`caching.drivetrain_tolerance`, `caching.xxx_tolerance`) in `config.yaml` to minimize bus overhead.

4. **Command-Based Subsystems**:
   - Use the `com.pedropathing.ivy.Command` API to structure actions (e.g., `runIntakeCommand`, `shootCommand`).
   - Define clear `.requiring(this)` constraints to prevent resource conflicts.

5. **Single-Writer Actuator Pattern**:
   - Every actuator has exactly one method that writes to hardware, and it is only ever called from that subsystem's `periodic()` — see `Shooter.periodic()` (writes flywheel velocity from `targetPower`), `Turret.periodic()` (writes servo/drive power from `AimMode`), and `Intake.periodic()` (a documented no-op; intake is a pure pass-through actuator).
   - `Robot.update()` is the only caller of subsystem `periodic()` methods, once per loop, in a fixed order (`Intake`, `Shooter`, `Turret`, `ShotController`).
   - Commands, `ShotController`, and OpModes must only set *target* state (`setTargetPower`, `setAimMode`, `setHoldAngle`, ...) — never call a raw hardware-writing method (e.g. a motor's `setPower`/`setVelocity`) from outside a subsystem's own `periodic()`. Direct writes that bypass `periodic()` get silently overwritten on the next loop and have caused real regressions (a manual-rev button that appeared to do nothing because `periodic()` reverted it one loop later).

6. **Dependency Injection Over Static Global State**:
   - Per-match state (current alliance, goal coordinates, starting poses) is threaded through explicit constructor parameters — `Sentinel(Alliance)`, `Casablanca(Sentinel)`, `Robot(HardwareMap, Telemetry, MatchProfile)` — not mutable public static fields.
   - Do not reintroduce a static "current alliance" or "current match" holder. A prior static-global design caused an initialization-ordering bug where `Sentinel` captured the wrong alliance depending on which OpMode constructed it first; constructor injection makes that class of bug structurally impossible and makes the safety/geometry classes independently unit-testable.

7. **Safety & Collision Avoidance**:
   - Do not bypass `Sentinel` or `Casablanca` protections during automated or manual driving sequences.
   - `Sentinel` and `Casablanca` are per-match instances owned by `Robot` (see rule 6) — always get them via `robot.getSentinel()` / `robot.getCasablanca()`, never construct a second instance for the same match.
   - Any modifications to the safety calculations should utilize Android `Path`/geometry checks (see rule 2's testability exception for the SAT fallback), and depth (X-axis) vs. side (Y-axis) protection must stay symmetric — if you change one axis's bounds wiring in `Casablanca.adjustDriveInput`, mirror it in the other.

8. **Modern Java Features**:
   - Prefer modern Java syntax like `switch` expressions and `record` types for simple data containers (e.g. `LaunchParameters`, `MatchProfile`, `PIDGains` inside the `records` package).

9. **Autotuning & Calibration**:
   - For relay-based PID autotuning, extend `AutotuneOpMode` (which wraps `PIDAutotuner`) rather than reimplementing the oscillation/telemetry loop — see `HeadingAutotuneOpMode` and `TurretAutotuneOpMode` for the three methods to implement (`createAutotuner`, `readCurrentValue`, `applyPower`).
   - For non-PID calibration (e.g. finding a friction/kS constant via a power ramp), model the OpMode after `FrictionCalibrationOpMode` instead — it is not a `PIDAutotuner` use case.

10. **Mathematical & Algorithmic Unit Test Coverage**:
    - High overall global code coverage (e.g., 80% across the entire codebase) is not required.
    - However, all pure mathematical functions, geometry collision calculations (e.g., coordinate rotations, bounding box intersections in `Sentinel`/`Casablanca`), and numeric helpers (e.g., yaw angle wrapping in `Turret`) **must** have full local JVM unit test coverage to prevent logic regressions. See `MathSafetyTest` for the current baseline (turret angle math, Casablanca friction/slew/lane-fade math, Sentinel footprint rotation and intersection, Casablanca depth/side axis symmetry).

11. **Verification Task Workflow**:
    - Prefer running the custom task `./gradlew verifyBuild` to run the exact same verification checks as a standard build (Spotless auto-formatting, compiler error checks, local JVM tests, and the `checkConfigKeys` config-symmetry check) without deploying the application.
    - Always run `./gradlew` commands outside the terminal sandbox (`BypassSandbox: true`) because Gradle requires access to global cache lock files (e.g. `~/.gradle/wrapper/dists/...`) which are restricted by standard sandboxing.
    - For live tuning on a connected robot without a full APK rebuild, use `./gradlew pushConfig` to hot-deploy `config.yaml` over ADB (re-init the OpMode to pick it up) and `./gradlew resetConfig` to revert to the bundled config. This is a deployment convenience, not part of `verifyBuild`.

## Context

### architecture

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
   - `Sentinel` checks robot footprint geometry against protected alliance goal zones using JTS Topology Suite (`org.locationtech.jts.geom.Polygon.intersects()`). This provides robust, instant 2D polygon collision detection under pure JUnit 4 unit tests without requiring Android Robolectric wrappers.
   - `Casablanca` dynamically reduces velocity or repels the robot to prevent violating opponent goals, using a `PredictiveBrakingController` to gauge required stopping distance. Depth (X-axis) and side (Y-axis) protection are computed by the same `calculateAxisState` helper called twice with the appropriately-swapped bounds — keep the axis pairing symmetric when touching this code (see `MathSafetyTest#testCasablancaSymmetryAndCollisionMath`).

6. **Alliance-Parameterized OpModes**:
   - There is one `TeleOp.java`, not per-alliance TeleOp files. It reads `Alliance` from the blackboard (falling back to `RED`), builds a `MatchProfile` via `config.loadMatchProfile(alliance)`, and constructs `Robot` from it.
   - Autonomous logic lives in two alliance-agnostic abstract classes, `AllianceAutoNew` and `AllianceOppositeNew`, each taking an `Alliance` in its constructor and loading its poses via `config` (see block 8).
   - `UnifiedAutos.java` registers the four concrete OpModes (`Blue/Red` × `AutoNew/OppositeNew`) programmatically via `@OpModeRegistrar`, as thin one-line subclasses — there are no more hand-duplicated per-alliance auto files.
   - Each auto/teleop OpMode persists its final pose to the blackboard (`"RED_POSE"` / `"BLUE_POSE"`) so the next OpMode (typically TeleOp) can resume from the correct field position.

7. **Dynamic Configuration System**:
   - Config lives in one YAML file, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml`, with human-readable descriptions and min/max constraints kept separately in the sibling `config-docs.yaml` (keyed by leaf name).
   - Runtime loading is handled by the `config-compiler` Gradle module (`org.firstinspires.ftc.teamcode.config.ConfigLoader`), a small reflection-based YAML→POJO binder. It prefers an ADB-pushed override at `/sdcard/FIRST/teamcode/config.yaml` over the bundled classpath resource, so config can be hot-reloaded without an APK rebuild (`./gradlew pushConfig` / `./gradlew resetConfig`, backed by `scripts/push_config.py`).
   - `config.java` (under `robot/config/`) is a **generated** typed facade over that loader — nested static classes mirroring the YAML structure — and a `loadMatchProfile(Alliance)` helper. It is regenerated from `config.yaml` + `config-docs.yaml` by the pure Java generator `ConfigGeneratorMain` (`./gradlew generateConfig`), which also regenerates `config-schema.json`; never hand-edit `config.java` or `config-schema.json`.
   - `checkConfigKeys` (wired into the `checkConfigKeys` Gradle task, part of `verifyBuild`) is a pure Java JavaExec task that statically checks `config.yaml` for alliance-name asymmetry (a `red_*` key with no `blue_*` mirror) and fuzzy near-duplicate keys, failing the build on the former.
   - The old `utilities/ConfigLoader.java` (string-path `getDouble`/`getInt`/... lookups) is `@Deprecated` and only kept as a thin shim over the new loader for backward compatibility — new code should read from `config`, not call it directly.

8. **Dedicated Records Package**:
   - The `org.firstinspires.ftc.teamcode.records` package contains simple, immutable record types and structural data elements:
     - `LaunchParameters`: Immutable shooter calculation parameters.
     - `PIDGains`: Holder for kp, ki, and kd gains.
     - `Alliance`: RED / BLUE enum.
     - `Field`: Dynamic goal coordinates derived from active alliance.
     - `MatchProfile`: Bundles `Alliance`, goal X/Y, and the four match poses (start/score/drink/park) for a given alliance; always passed into `Robot`'s constructor. `TeleOp` builds it via `config.loadMatchProfile(Alliance)`; `AllianceAutoNew`/`AllianceOppositeNew` build their own from the merged `auto_poses.*` config since their pose sets differ from TeleOp's. There is no static "current alliance/current match" global anywhere in the codebase — it is always threaded through explicitly via `MatchProfile`.
