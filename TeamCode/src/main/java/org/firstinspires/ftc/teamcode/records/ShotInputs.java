package org.firstinspires.ftc.teamcode.records;

import com.pedropathing.geometry.Pose;

/** Immutable live inputs fed into the stateless shot solver every control loop. */
public record ShotInputs(
    Pose robotPose, // Live localizer pose (X, Y, Heading in radians)
    Pose robotVelocity, // Live localizer velocity (Vx in/s, Vy in/s, Omega rad/s)
    double targetGoalX, // Target goal center X coordinate on field (inches)
    double targetGoalY // Target goal center Y coordinate on field (inches)
    ) {}
