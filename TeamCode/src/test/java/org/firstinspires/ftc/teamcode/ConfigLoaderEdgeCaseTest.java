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
  public void deepMerge_staleOverrideKeepsSectionsItDoesNotMention() {
    // Reproduces the failure that hard-crashed the robot app: an ADB-pushed config.yaml written
    // before turret.travel existed. Before the override was merged onto the bundled baseline it
    // replaced it outright, so turret.travel came back null and the generated facade NPE'd inside
    // a static initializer -- killing the app before any OpMode could run.
    java.util.Map<String, Object> bundled = new java.util.HashMap<>();
    java.util.Map<String, Object> bundledTurret = new java.util.HashMap<>();
    java.util.Map<String, Object> bundledTravel = new java.util.HashMap<>();
    bundledTravel.put("min_angle", -45.0);
    bundledTravel.put("max_angle", 45.0);
    bundledTurret.put("travel", bundledTravel);
    bundledTurret.put("ks", 0.08);
    bundled.put("turret", bundledTurret);

    // A stale push: has turret, tweaks one value, knows nothing about travel.
    java.util.Map<String, Object> stale = new java.util.HashMap<>();
    java.util.Map<String, Object> staleTurret = new java.util.HashMap<>();
    staleTurret.put("ks", 0.12);
    stale.put("turret", staleTurret);

    ConfigLoader.deepMerge(bundled, stale);

    @SuppressWarnings("unchecked")
    java.util.Map<String, Object> turret = (java.util.Map<String, Object>) bundled.get("turret");
    assertNotNull("stale override must not delete unmentioned sections", turret.get("travel"));

    @SuppressWarnings("unchecked")
    java.util.Map<String, Object> travel = (java.util.Map<String, Object>) turret.get("travel");
    org.junit.Assert.assertEquals(-45.0, (Double) travel.get("min_angle"), 1e-9);
    // ...while the value the override *did* specify still wins.
    org.junit.Assert.assertEquals(0.12, (Double) turret.get("ks"), 1e-9);
  }

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
