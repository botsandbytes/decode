package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

public class Mathb {
    public static double toRadians(double heading) {
        return MathFunctions.normalizeAngle(Math.PI - Math.toRadians(heading));
    }
}
