package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name = "Clock Test", group = "Test")
public class ClockTest extends OpMode {

  private final SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");

  @Override
  public void init() {
    telemetry.addData("Time", format.format(new Date()));
    telemetry.update();
  }

  @Override
  public void loop() {}
}
