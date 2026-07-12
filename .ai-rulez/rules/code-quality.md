---
priority: high
---

# Code Quality & Governance Standards

Follow these coding and design standards in the `decode` codebase:

## Coding Conventions
1. **Dynamic Configuration Over Hardcoding**:
   - Never hardcode mechanical coefficients, tolerances, PID constants, safety margins, velocity scales, or autonomous timing values in Java code.
   - Always add or reference them via `ConfigLoader` and define them in the appropriate config YAML file (`subsystems.yaml`, `safety.yaml`, `tuning.yaml`, or `auto.yaml` for autonomous-only constants).
   - Keep `config-schema.json` updated with accurate descriptions and constraints for new config options.

2. **Prefer Libraries Over Custom Implementations**:
   - Never write custom code for something a library can already do — even if that library is not yet installed. Identify and add the appropriate dependency first.
   - Use Android SDK types (`android.graphics.RectF`, `android.graphics.PointF`, `android.graphics.Path`) for 2D geometry instead of custom records.
   - Use Pedro Pathing's `com.pedropathing.control.PIDFController` and `PIDFCoefficients` instead of any custom PID/PIDF implementations.
   - Use FTC SDK's `org.firstinspires.ftc.robotcore.external.navigation.AngleUnit` normalization methods instead of custom modulo math.
   - This rule applies to: PID controllers, geometry types, angle math, data structures, interpolation, collections, and anything else with a well-supported library equivalent.

3. **Hardware Optimization via Dairy Caching**:
   - Drivetrain wheels, shooters, and rollers must use caching hardware wrappers (`CachingDcMotorEx`, `CachingServo`, `CachingCRServo`) to optimize loop times.
   - For the drivetrain specifically: wrap all four motors in `CachingDcMotorEx`, register them back into `hardwareMap` using `hardwareMap.put(name, cached)`, then call `Constants.createFollower(hardwareMap)` so Pedro Pathing automatically picks up the cached instances.
   - Set sensible tolerances (`caching.drivetrain_tolerance`, `caching.xxx_tolerance`) in `subsystems.yaml` to minimize bus overhead.

4. **Command-Based Subsystems**:
   - Use the `com.pedropathing.ivy.Command` API to structure actions (e.g., `runIntakeCommand`, `shootCommand`).
   - Define clear `.requiring(this)` constraints to prevent resource conflicts.

5. **Safety & Collision Avoidance**:
   - Do not bypass `Sentinel` or `Casablanca` protections during automated or manual driving sequences.
   - Any modifications to the safety calculations should utilize Android `Path` geometry checks.

6. **Modern Java Features**:
   - Prefer modern Java syntax like `switch` expressions and `record` types for simple data containers (e.g. `LaunchParameters`).

7. **Autotuning & Calibration**:
   - When introducing parameters that require live calibration, inherit from `PIDAutotuner` or implement tests modeled after `HeadingAutotuneOpMode` and `FrictionCalibrationOpMode`.

