package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import java.lang.reflect.Field;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.Test;

/**
 * Covers the ConfigLoader "mirror" system (an {@code m}/{@code mirror} value means "derive this
 * from the opposite alliance's value via {@link Pose#mirror()}"). {@code auto_poses.opposite.red}
 * and {@code auto_poses.normal.red} are now explicit, field-tuned poses (pulled from the historical
 * Red autos) rather than {@code m} placeholders, because they are not exact geometric mirrors of
 * blue (e.g. opposite start x=87 vs. mirror-of-blue x=84.5) — driving/vision asymmetry meant the
 * two alliances were tuned separately on the field. The generic mirror-evaluation utility itself
 * (used elsewhere, and by scalar/non-pose keys) is still covered below.
 */
public class ConfigLoaderMirrorTest {

  private static final double EPS = 1e-4;

  /**
   * The explicit opposite-RED poses must be defined and must NOT be exact mirrors of blue — if they
   * ever collapse onto the mirror of blue, that's a sign the config regressed back to a bare {@code
   * m} placeholder instead of the hand-tuned values.
   */
  @Test
  public void oppositeRed_posesAreExplicit_notExactMirrorOfBlue() throws Exception {
    config.OppositeAuto blue =
        ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite.blue", "auto");
    config.OppositeAuto red =
        ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite.red", "auto");

    int checked = 0;
    boolean anyDiffersFromMirror = false;
    for (Field f : config.OppositeAuto.class.getFields()) {
      if (f.getType() != Pose.class) continue;
      Pose bp = (Pose) f.get(blue);
      Pose rp = (Pose) f.get(red);
      assertNotNull("blue " + f.getName() + " should be explicitly defined", bp);
      assertNotNull("red " + f.getName() + " should be explicitly defined", rp);

      Pose mirrored = bp.mirror();
      if (Math.abs(mirrored.getX() - rp.getX()) > EPS
          || Math.abs(mirrored.getY() - rp.getY()) > EPS) {
        anyDiffersFromMirror = true;
      }
      checked++;
    }
    assertTrue("expected all opposite-auto poses to be checked", checked >= 10);
    assertTrue(
        "expected at least one opposite-RED pose to be a hand-tuned value, not an exact mirror"
            + " of blue",
        anyDiffersFromMirror);
  }

  /** Direct {@code load(Pose.class, path)} on the explicit red start pose returns real values. */
  @Test
  public void directLoadPath_resolvesExplicitRedPose() {
    Pose blue = ConfigLoader.load(Pose.class, "auto_poses.opposite.blue.start");
    Pose red = ConfigLoader.load(Pose.class, "auto_poses.opposite.red.start");
    assertNotNull(blue);
    assertNotNull(red);
    // Historical Red Opposite start pose (RedOppositeNew.java): (87, 8, 90deg).
    assertEquals(87.0, red.getX(), EPS);
    assertEquals(8.0, red.getY(), EPS);
  }

  /**
   * The explicit red start pose is close to, but not exactly, the mirror of blue's start pose —
   * confirming these were tuned per-alliance rather than derived.
   */
  @Test
  public void explicitRedPose_isNotPassthroughOfMirror() {
    Pose blue = ConfigLoader.load(Pose.class, "auto_poses.opposite.blue.start");
    Pose red = ConfigLoader.load(Pose.class, "auto_poses.opposite.red.start");
    assertNotEquals("red x should differ from blue x", blue.getX(), red.getX(), 1e-6);
    assertNotEquals(
        "red x should not equal the exact Pedro mirror (field length 141.5) of blue x",
        141.5 - blue.getX(),
        red.getX(),
        EPS);
  }

  /** Non-alliance pair math (high/low): verify "k+0.5" adds the offset to the partner's value. */
  @Test
  public void nonAlliancePair_mathResolution() throws Exception {
    java.lang.reflect.Method eval =
        ConfigLoader.class.getDeclaredMethod(
            "evaluateMirrorOperation", Object.class, Object.class, Class.class, String.class);
    eval.setAccessible(true);

    // Sample: base low_position = 0.1 → high_position with "k+0.5" should be 0.6
    double sampleBase = 0.1;
    assertEquals(0.6, (Double) eval.invoke(null, sampleBase, "k+0.5", double.class, "dummy"), EPS);

    // Sample: base = 0.3 → "k-0.1" should give 0.2
    assertEquals(0.2, (Double) eval.invoke(null, 0.3, "k-0.1", double.class, "dummy"), EPS);

    // Sample: base = 0.4 → "k*2" should give 0.8
    assertEquals(0.8, (Double) eval.invoke(null, 0.4, "k*2", double.class, "dummy"), EPS);

    // Sample: base = 0.6 → "k/2" should give 0.3
    assertEquals(0.3, (Double) eval.invoke(null, 0.6, "k/2", double.class, "dummy"), EPS);
  }

  @Test
  public void mirrorMathOperations_mAndShorthandForms() throws Exception {
    java.lang.reflect.Method eval =
        ConfigLoader.class.getDeclaredMethod(
            "evaluateMirrorOperation", Object.class, Object.class, Class.class, String.class);
    eval.setAccessible(true);

    // For scalars, 'm' is identity (no spatial mirror), so m+N = base+N
    // Sample: base=0.5 → "m+0.5" gives 1.0, shorthand "+0.5" also gives 1.0
    double base = 0.5;
    assertEquals(1.0, (Double) eval.invoke(null, base, "m+0.5", double.class, "dummy"), EPS);
    assertEquals(1.0, (Double) eval.invoke(null, base, "+0.5", double.class, "dummy"), EPS);
    // Sample: base=0.5 → "m*2" / "*2" gives 1.0
    assertEquals(1.0, (Double) eval.invoke(null, base, "*2", double.class, "dummy"), EPS);
    // Sample: base=0.8 → "*2" gives 1.6
    assertEquals(1.6, (Double) eval.invoke(null, 0.8, "*2", double.class, "dummy"), EPS);
  }

  @Test
  public void mirrorVsKey_poseTransformations() throws Exception {
    java.lang.reflect.Method eval =
        ConfigLoader.class.getDeclaredMethod(
            "evaluateMirrorOperation", Object.class, Object.class, Class.class, String.class);
    eval.setAccessible(true);

    Pose basePose = new Pose(27.0, 128.0, 135.0);
    Pose expectedMirror = basePose.mirror();

    // 'm' mirrors the pose
    Pose mirrored = (Pose) eval.invoke(null, basePose, "m", Pose.class, "dummy");
    assertEquals(expectedMirror.getX(), mirrored.getX(), EPS);
    assertEquals(expectedMirror.getY(), mirrored.getY(), EPS);

    // 'm+2' mirrors the pose first, then adds 2
    Pose mirroredPlusTwo = (Pose) eval.invoke(null, basePose, "m+2", Pose.class, "dummy");
    assertEquals(expectedMirror.getX() + 2.0, mirroredPlusTwo.getX(), EPS);
    assertEquals(expectedMirror.getY() + 2.0, mirroredPlusTwo.getY(), EPS);

    // 'k' keeps the original pose UN-MIRRORED
    Pose original = (Pose) eval.invoke(null, basePose, "k", Pose.class, "dummy");
    assertEquals(basePose.getX(), original.getX(), EPS);
    assertEquals(basePose.getY(), original.getY(), EPS);

    // 'k+2' keeps original pose and adds 2
    Pose originalPlusTwo = (Pose) eval.invoke(null, basePose, "k+2", Pose.class, "dummy");
    assertEquals(basePose.getX() + 2.0, originalPlusTwo.getX(), EPS);
    assertEquals(basePose.getY() + 2.0, originalPlusTwo.getY(), EPS);
  }
}
