package org.firstinspires.ftc.teamcode.records;

import com.pedropathing.geometry.Pose;

/**
 * Where the robot should sit out the end of autonomous, and which side of the launch zone boundary
 * it committed to.
 *
 * <p>{@code insideLaunchZone} is not a detail for telemetry: a robot parked inside a launch zone is
 * a robot that can keep shooting for the seconds it has left, and one parked outside must not.
 *
 * @see org.firstinspires.ftc.teamcode.utilities.Sentinel#nearestEndgameSpot
 */
public record EndgameSpot(Pose pose, boolean insideLaunchZone) {}
