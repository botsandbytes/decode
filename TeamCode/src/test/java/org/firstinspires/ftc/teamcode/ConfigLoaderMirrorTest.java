package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import java.lang.reflect.Field;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.robot.config.config;
import org.junit.Test;

/**
 * Covers the ConfigLoader "mirror" system (an {@code m}/{@code mirror} value means "derive this
 * from the opposite alliance's value via {@link Pose#mirror()}"). The entire {@code
 * auto_poses.opposite.red} routine is defined this way, so this is competition-critical geometry
 * that previously had no test coverage.
 */
public class ConfigLoaderMirrorTest {

  private static final double EPS = 1e-4;

  /** Every mirrored opposite-RED pose must equal the mirror of the explicit opposite-BLUE pose. */
  @Test
  public void oppositeRed_allPosesAreMirrorOfBlue() throws Exception {
    config.OppositeAuto blue =
        ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite.blue", "auto");
    config.OppositeAuto red =
        ConfigLoader.loadMerged(config.OppositeAuto.class, "auto_poses.opposite.red", "auto");

    int checked = 0;
    for (Field f : config.OppositeAuto.class.getFields()) {
      if (f.getType() != Pose.class) continue;
      Pose bp = (Pose) f.get(blue);
      Pose rp = (Pose) f.get(red);
      assertNotNull("blue " + f.getName() + " should be explicitly defined", bp);
      assertNotNull("red " + f.getName() + " should resolve via mirror", rp);

      Pose expected = bp.mirror();
      assertEquals("mirror x for " + f.getName(), expected.getX(), rp.getX(), EPS);
      assertEquals("mirror y for " + f.getName(), expected.getY(), rp.getY(), EPS);
      assertEquals(
          "mirror heading for " + f.getName(), expected.getHeading(), rp.getHeading(), EPS);
      checked++;
    }
    assertTrue("expected all opposite-auto poses to be checked", checked >= 10);
  }

  /** The direct {@code load(Pose.class, path)} path (not loadMerged) must also resolve mirrors. */
  @Test
  public void directLoadPath_resolvesMirror() {
    Pose blue = ConfigLoader.load(Pose.class, "auto_poses.opposite.blue.start");
    Pose red = ConfigLoader.load(Pose.class, "auto_poses.opposite.red.start");
    assertNotNull(blue);
    assertNotNull(red);
    assertEquals(blue.mirror().getX(), red.getX(), EPS);
    assertEquals(blue.mirror().getY(), red.getY(), EPS);
    assertEquals(blue.mirror().getHeading(), red.getHeading(), EPS);
  }

  /**
   * A mirror must actually transform the value (x' = 141.5 - x), not pass it through or null it.
   */
  @Test
  public void mirror_transformsValue_notPassthrough() {
    Pose blue = ConfigLoader.load(Pose.class, "auto_poses.opposite.blue.start");
    Pose red = ConfigLoader.load(Pose.class, "auto_poses.opposite.red.start");
    assertNotEquals("mirror should change x", blue.getX(), red.getX(), 1e-6);
    assertEquals("Pedro mirror uses field length 141.5", 141.5 - blue.getX(), red.getX(), EPS);
  }

  /**
   * Guard against over-triggering: {@code normal.red} is explicitly defined (no {@code m}), so it
   * must load its own literal values, NOT the mirror of {@code normal.blue}.
   */
  @Test
  public void explicitSection_isNotMirrorDerived() {
    config.NormalAuto blue =
        ConfigLoader.loadMerged(config.NormalAuto.class, "auto_poses.normal.blue", "auto");
    config.NormalAuto red =
        ConfigLoader.loadMerged(config.NormalAuto.class, "auto_poses.normal.red", "auto");

    // normal.red.start is the literal [117, 128, 45]; blue.start is [27, 128, 135].
    assertEquals("explicit red start must be its own literal x", 117.0, red.start.getX(), EPS);
    assertNotEquals(
        "explicit section must not be mirror-derived",
        blue.start.mirror().getX(),
        red.start.getX(),
        EPS);
  }
}
