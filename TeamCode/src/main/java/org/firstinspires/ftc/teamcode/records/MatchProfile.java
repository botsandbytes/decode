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

  public static MatchProfile loadMatchProfile(Alliance alliance) {
    String allianceStr = alliance == Alliance.RED ? "red" : "blue";
    double goalX = Field.getGoalX(alliance);
    double goalY = Field.getGoalY(alliance);
    Pose startPose =
        org.firstinspires.ftc.teamcode.config.ConfigLoader.load(
            Pose.class, "teleop.poses." + allianceStr + ".start");
    Pose drinkPose =
        org.firstinspires.ftc.teamcode.config.ConfigLoader.load(
            Pose.class, "teleop.poses." + allianceStr + ".drink");
    Pose parkPose =
        org.firstinspires.ftc.teamcode.config.ConfigLoader.load(
            Pose.class, "teleop.poses." + allianceStr + ".park");
    Pose scorePose =
        org.firstinspires.ftc.teamcode.config.ConfigLoader.load(
            Pose.class, "teleop.poses." + allianceStr + ".score");
    return new MatchProfile(alliance, goalX, goalY, startPose, drinkPose, parkPose, scorePose);
  }
}
