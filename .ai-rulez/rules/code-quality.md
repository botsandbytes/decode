---
priority: high
---

# Code Quality & Governance Standards

Follow these coding and design standards in the `decode` codebase:

## Coding Conventions
1. **Dynamic Configuration Over Hardcoding**:
   - Never hardcode mechanical coefficients, tolerances, PID constants, safety margins, velocity scales, or autonomous timing values in Java code.
   - Add or change values in `config.yaml`; access them in Java via the generated `config` facade (e.g. `config.turret.pidf.p`) — do not call the low-level `config.ConfigLoader.load(...)` directly from subsystem code, and never call the deprecated `utilities.ConfigLoader`.
   - When adding a new config key, also add its description (and `min`/`max` where applicable) to `config-docs.yaml`, then run `python3 scripts/generate_config.py` to regenerate `config.java` and `config-schema.json`. Never hand-edit either generated file.
   - If a new key mentions an alliance (`red_*`/`blue_*`), add its mirror for the other alliance or the `checkConfigKeys` Gradle task (part of `verifyBuild`) will fail the build.

2. **Prefer Libraries Over Custom Implementations**:
   - Never write custom code for something a library can already do — even if that library is not yet installed. Identify and add the appropriate dependency first.
   - Use Android SDK types (`android.graphics.RectF`, `android.graphics.PointF`, `android.graphics.Path`) for 2D geometry instead of custom records.
   - Use Pedro Pathing's `com.pedropathing.control.PIDFController` and `PIDFCoefficients` instead of any custom PID/PIDF implementations.
   - Use FTC SDK's `org.firstinspires.ftc.robotcore.external.navigation.AngleUnit` normalization methods instead of custom modulo math.
   - This rule applies to: PID controllers, geometry types, angle math, data structures, interpolation, collections, and anything else with a well-supported library equivalent.
   - Exception: `Sentinel`'s polygon intersection falls back to a hand-written Separating Axis Theorem check only because `android.graphics.Path.op()` isn't implemented under Robolectric/JVM tests. That fallback exists solely to satisfy rule 10's test-coverage requirement — it is not license to duplicate other library functionality for testability elsewhere. Prefer making the real dependency testable before reaching for a parallel implementation.

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
    - Prefer running the custom task `./gradlew verifyBuild` to run the exact same verification checks as a standard build (Spotless auto-formatting, compiler error checks, SpotBugs analysis, local JVM tests, and the `checkConfigKeys` config-symmetry check) without deploying the application.
    - Always run `./gradlew` commands outside the terminal sandbox (`BypassSandbox: true`) because Gradle requires access to global cache lock files (e.g. `~/.gradle/wrapper/dists/...`) which are restricted by standard sandboxing.
    - For live tuning on a connected robot without a full APK rebuild, use `./gradlew pushConfig` to hot-deploy `config.yaml` over ADB (re-init the OpMode to pick it up) and `./gradlew resetConfig` to revert to the bundled config. This is a deployment convenience, not part of `verifyBuild`.
