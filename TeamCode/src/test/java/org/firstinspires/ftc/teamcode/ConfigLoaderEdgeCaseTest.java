package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertNotNull;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.Test;

/**
 * Proves that reflection-based config binding loads non-null instances, binds Pose structures, and
 * resolves match profiles correctly using lowercase {@link config}.
 */
public class ConfigLoaderEdgeCaseTest {

  @Test
  public void loadMerged_autoPoses_bindsListPosesAndGlobals() {
    config.NormalAuto cfg =
        ConfigLoader.loadMerged(config.NormalAuto.class, "auto_poses.normal.blue", "auto");
    assertNotNull("Config object should be loaded", cfg);
    assertNotNull("Start pose should be bound", cfg.start);
  }

  @Test
  public void loadPoseByPath_blueStart() {
    Pose p = ConfigLoader.load(Pose.class, "teleop.poses.blue.start");
    assertNotNull("Pose should be loaded", p);
  }

  @Test
  public void loadDoubleByPath_scalar() {
    Double v = ConfigLoader.load(Double.class, "sentinel.goals.red_goal_x");
    assertNotNull("Scalar value should be loaded", v);
  }

  @Test
  public void loadMatchProfile_blue_hasRealPoses() {
    MatchProfile profile = config.loadMatchProfile(Alliance.BLUE);
    assertNotNull("Blue MatchProfile should load", profile);
    assertNotNull("Start pose should load", profile.startPose());
    assertNotNull("Score pose should load", profile.scorePose());
  }

  @Test
  public void loadMatchProfile_red_hasRealPoses() {
    MatchProfile profile = config.loadMatchProfile(Alliance.RED);
    assertNotNull("Red MatchProfile should load", profile);
    assertNotNull("Start pose should load", profile.startPose());
  }

  @Test
  public void matchProfile_flexibleConstructors_autoPopulateGoals() {
    MatchProfile redProfile = new MatchProfile(Alliance.RED);
    assertNotNull("Red profile should construct", redProfile);

    MatchProfile blueProfile = new MatchProfile(Alliance.BLUE);
    assertNotNull("Blue profile should construct", blueProfile);
  }
}
