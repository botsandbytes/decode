package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.junit.Test;

/**
 * Covers the latching feed gate in {@link ShotController#shouldFeed}. A ring entering the flywheel
 * drags its speed down, so a gate with a single hard threshold cuts the intake mid-feed, lets the
 * wheel recover, and re-opens — the intake visibly stutters instead of running continuously. These
 * pin the hysteresis that prevents that.
 */
public class FeedGateHysteresisTest {

  // Live config.yaml values.
  private static final double ARM_LOW = 0.95;
  private static final double ARM_HIGH = 1.05;
  private static final double RELEASE_LOW = 0.80;

  private static boolean feed(double ratio, boolean feeding) {
    return ShotController.shouldFeed(ratio, feeding, ARM_LOW, ARM_HIGH, RELEASE_LOW);
  }

  @Test
  public void testGateStaysShutDuringSpinUp() {
    assertFalse("well below target must not feed", feed(0.50, false));
    assertFalse("just under the arm floor must not feed", feed(0.94, false));
  }

  @Test
  public void testGateArmsInsideWindow() {
    assertTrue("at the arm floor", feed(ARM_LOW, false));
    assertTrue("on target", feed(1.00, false));
    assertTrue("at the arm ceiling", feed(ARM_HIGH, false));
  }

  @Test
  public void testOverspeedBlocksArming() {
    assertFalse("above the arm ceiling must not start a feed", feed(1.06, false));
  }

  @Test
  public void testFeedSurvivesRingSag() {
    // The whole point: a ring pulls the wheel under the arm floor, and the feed must continue.
    assertTrue("sag below arm floor keeps feeding", feed(0.90, true));
    assertTrue("deeper sag still keeps feeding", feed(0.81, true));
    assertTrue("exactly at the release floor keeps feeding", feed(RELEASE_LOW, true));
  }

  @Test
  public void testFeedReleasesWhenFlywheelTrulyStalls() {
    assertFalse("below the release floor cuts the feed", feed(0.79, true));
    assertFalse("stalled wheel cuts the feed", feed(0.10, true));
  }

  @Test
  public void testOverspeedDoesNotInterruptRunningFeed() {
    // Rings only ever slow the wheel, so an over-speed reading mid-feed is the controller
    // overshooting on recovery. Cutting the feed for it would stutter from the other direction.
    assertTrue("overshoot while feeding must not cut the feed", feed(1.20, true));
  }

  @Test
  public void testReleaseFloorIsBelowArmFloorSoBandIsRealHysteresis() {
    assertTrue(
        "release floor must sit below the arm floor or there is no hysteresis at all",
        RELEASE_LOW < ARM_LOW);

    // A ratio inside the band resolves purely on prior state — that is the hysteresis.
    double insideBand = 0.90;
    assertFalse("inside the band, a shut gate stays shut", feed(insideBand, false));
    assertTrue("inside the band, an open gate stays open", feed(insideBand, true));
  }
}
