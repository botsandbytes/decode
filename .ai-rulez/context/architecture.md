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
     - `Shooter.java`: Double flywheel and hood positioning.
     - `Turret.java`: CRServo-driven turret rotation with dedicated IMU yaw feedback.
   - Subsystem behaviors are orchestrated using the `com.pedropathing.ivy.Command` command framework.

2. **Drivetrain & Pathing (Pedro Pathing)**:
   - Drivetrain control, localizer, and path-following are managed by **Pedro Pathing**.
   - Custom constants (mass, PIDF, predictive braking) are configured in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`.

3. **Hardware Abstraction & Performance (dev.frozenmilk Dairy)**:
   - Utilizes the **Dairy** framework (Core, Pasteurized, CachingHardware, Sloth) for optimization.
   - Drivetrain motors, shooter flywheels, and intake rollers use `dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx` to cache write commands, significantly reducing redundant I2C writes and loop times.

4. **Collision Avoidance (Casablanca & Sentinel)**:
   - `Sentinel.java` checks robot footprint geometry against protected alliance goal zones using standard Android `android.graphics.Path` intersections.
   - `Casablanca.java` dynamically reduces velocity or repels the robot to prevent violating opponent goals, using a `PredictiveBrakingController` to gauge required stopping distance.

5. **Dynamic Configuration Loader**:
   - Subsystem parameters, safety thresholds, and PIDF constants are stored in classpath resource YAML files under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/`:
     - `subsystems.yaml`
     - `safety.yaml`
     - `tuning.yaml`
   - Configs are dynamically merged and queried at runtime via `ConfigLoader.java` using SnakeYAML, matching `config-schema.json`.
