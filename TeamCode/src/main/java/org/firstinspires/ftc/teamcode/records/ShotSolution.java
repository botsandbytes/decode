package org.firstinspires.ftc.teamcode.records;

/** Immutable outcome of the stateless shot solver for a given loop. */
public record ShotSolution(
    double targetHoodPosition, // Continuous hood servo position (0.0 to 1.0)
    double targetRpm, // Solved flywheel setpoint for this shot
    double targetExitVelocityIps, // Solved ring exit speed the setpoint should produce (in/s)
    double targetAzimuthRad, // Field-frame target bearing (radians)
    double predictedFlightTimeSec, // Integrated time of flight (seconds)
    double predictedEntryAngleRad, // Integrated trajectory landing angle (radians below horizontal)
    double distanceInches, // Virtual distance used for solution (inches)
    boolean isValid, // Solution validity flag (converged & within speed/range/reachability gates)
    String validityReason // Human-readable reason if invalid
    ) {}
