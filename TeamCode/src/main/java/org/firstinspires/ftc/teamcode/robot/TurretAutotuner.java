package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.utilities.PIDAutotuner;

public class TurretAutotuner extends PIDAutotuner {

  public void startAutotune(double currentAngle, double outputMagnitude) {
    super.startAutotune(currentAngle, currentAngle, outputMagnitude);
  }
}
