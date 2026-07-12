---
priority: high
---

# Code Quality & Governance Standards

Follow these coding and design standards in the `decode` codebase:

## Coding Conventions
1. **Dynamic Configuration Over Hardcoding**:
   - Never hardcode mechanical coefficients, tolerances, PID constants, safety margins, or velocity scales in Java code.
   - Always add or reference them via `ConfigLoader` and define them in the appropriate config YAML file (`subsystems.yaml`, `safety.yaml`, `tuning.yaml`).
   - Keep `config-schema.json` updated with accurate descriptions and constraints for new config options.

2. **Hardware Optimization via Dairy Caching**:
   - Drivetrain wheels, shooters, and rollers must use caching hardware wrappers (`CachingDcMotorEx`, `CachingServo`, `CachingCRServo`) to optimize loop times.
   - Set sensible tolerances (`caching.xxx_tolerance`) in `subsystems.yaml` to minimize bus overhead.

3. **Command-Based Subsystems**:
   - Use the `com.pedropathing.ivy.Command` API to structure actions (e.g., `runIntakeCommand`, `shootCommand`).
   - Define clear `.requiring(this)` constraints to prevent resource conflicts.

4. **Safety & Collision Avoidance**:
   - Do not bypass `Sentinel` or `Casablanca` protections during automated or manual driving sequences.
   - Any modifications to the safety calculations should utilize Android `Path` geometry checks.

5. **Modern Java Features**:
   - Prefer modern Java syntax like `switch` expressions and `record` types for simple data containers (e.g. `LaunchParameters`).

6. **Autotuning & Calibration**:
   - When introducing parameters that require live calibration, inherit from `PIDAutotuner` or implement tests modeled after `HeadingAutotuneOpMode` and `FrictionCalibrationOpMode`.
