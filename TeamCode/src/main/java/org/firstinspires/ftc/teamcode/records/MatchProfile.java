package org.firstinspires.ftc.teamcode.records;

import com.pedropathing.geometry.Pose;

public record MatchProfile(
    Alliance alliance,
    double goalX,
    double goalY,
    Pose startPose,
    Pose drinkPose,
    Pose parkPose,
    Pose scorePose) {

  public MatchProfile(Alliance alliance) {
    this(alliance, Field.getGoalX(alliance), Field.getGoalY(alliance), null, null, null, null);
  }

  public MatchProfile(
      Alliance alliance, Pose startPose, Pose drinkPose, Pose parkPose, Pose scorePose) {
    this(
        alliance,
        Field.getGoalX(alliance),
        Field.getGoalY(alliance),
        startPose,
        drinkPose,
        parkPose,
        scorePose);
  }
}
