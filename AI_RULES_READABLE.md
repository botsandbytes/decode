# AI Agents Rule Governance Manifest

This document outlines the master rules, coding limits, behavioral requirements, and architectural constraints pushed to our AI coding assistants (Claude, Copilot, Codex, and Antigravity) by the `ai-rulez` compilation system.

---

## 1. Core Behavioral Rules

All AI assistants must adhere to the following core guidelines:

* **Source of Truth Priority**: The source of truth for all AI instructions resides in the `.ai-rulez/` directory. Assistants are instructed *never* to manually modify generated rule files like `AGENTS.md`, `CLAUDE.md`, or `.github/copilot-instructions.md`.
* **Dynamic Config Verification**: Before adding new mechanical powers, tolerances, or coefficients, agents must ensure they define them inside the configuration YAML resource blocks instead of hardcoding.
* **State Caching Compliance**: When configuring motors, servos, or encoders, agents must check if they are wrapped by Dairy's caching hardware interfaces.

---

## 2. Engineering Coding Conventions

The following constraints are pushed to AI agents:

| Rule Category | Engineering Target | Constraints & Instructions |
| :--- | :--- | :--- |
| **Constants & Gains** | Subsystem Configuration | Never hardcode coefficients, PIDF values, or physical measurements. All settings must be queried through `ConfigLoader` from YAML files. |
| **Hardware Abstraction** | I2C Optimization | Always wrap raw drivetrain and flywheel motors using `CachingDcMotorEx` or `CachingServo` to reduce bus latency. |
| **Logic Structure** | Commands & Tasks | Subsystem movements must be modeled as Ivy Command actions, specifying explicit subsystem requirements (`.requiring(this)`). |
| **Safety Integration** | Collision Avoidance | All path safety updates must preserve Android `Path` geometry checks under `Sentinel` and `Casablanca`. |
| **Code Style** | Modern Java Features | Prefer utilizing modern Java constructs such as modern switch expressions and records for parameter wrappers. |
| **Calibration** | Parameter Tuning | For dynamic values, build automated calibration linear tests (extending `PIDAutotuner` or modeling opmodes). |

---

## 3. Reference Architecture Constraints

Agents are constrained to respect our specific subsystem footprint mapping:

### A. Subsystems & Responsibilities
* **Intake Subsystem**: Manages front and middle rollers. Interfaces with Ivy Command scheduler.
* **Shooter Subsystem**: Governs double flywheel RPM speeds and hood positioning. Syncs with Intake.
* **Turret Subsystem**: Coordinates yaw rotation using a dedicated IMU and CRServo. Syncs target angles relative to robot heading and goal locations.

### B. Geometry & Safety Limits
* Opponent goal zones are strictly protected. The robot must use Sentinel's graphic path checker to guarantee it never intersects the active opponent goal zone.
* Casablanca must use a `PredictiveBrakingController` leveraging constants to calculate deceleration bounds and scale drivetrain velocities dynamically.
