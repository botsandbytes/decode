package org.firstinspires.ftc.teamcode.records;

import org.firstinspires.ftc.teamcode.robot.config.config;

public class Field {
  // Not static final: config.sentinel is rebuilt on config.reload(), and these must reflect that
  // rebuild rather than freeze the value from the first time Field is touched.
  public static double getRedGoalX() {
    return config.sentinel.goals.red_goal_x;
  }

  public static double getRedGoalY() {
    return config.sentinel.goals.red_goal_y;
  }

  public static double getBlueGoalX() {
    return config.sentinel.goals.blue_goal_x;
  }

  public static double getBlueGoalY() {
    return config.sentinel.goals.blue_goal_y;
  }

  public static double getGoalX(Alliance alliance) {
    return alliance == Alliance.RED ? getRedGoalX() : getBlueGoalX();
  }

  public static double getGoalY(Alliance alliance) {
    return alliance == Alliance.RED ? getRedGoalY() : getBlueGoalY();
  }
}
