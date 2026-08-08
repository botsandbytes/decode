package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.junit.Test;

public class ShotTableTest {

  /** distance, hood servo, rpm */
  private static final double[] POINTS = {
    48.0, 0.100, 900.0,
    72.0, 0.200, 1000.0,
    96.0, 0.300, 1100.0,
    120.0, 0.400, 1200.0
  };

  private static ShotTable table() {
    return ShotTable.fromPoints(POINTS);
  }

  @Test
  public void testExactRowsReturnedUnchanged() {
    ShotTable t = table();
    for (int i = 0; i < POINTS.length; i += 3) {
      ShotTable.Shot s = t.lookup(POINTS[i]);
      assertEquals(
          "hood at calibrated row " + POINTS[i], POINTS[i + 1], s.hoodServoPosition(), 1e-9);
      assertEquals("rpm at calibrated row " + POINTS[i], POINTS[i + 2], s.rpm(), 1e-9);
      assertTrue("a calibrated row is in range", s.withinCalibratedRange());
    }
  }

  @Test
  public void testLinearInterpolationBetweenRows() {
    ShotTable t = table();
    ShotTable.Shot mid = t.lookup(60.0); // halfway between 48 and 72
    assertEquals(0.150, mid.hoodServoPosition(), 1e-9);
    assertEquals(950.0, mid.rpm(), 1e-9);
    assertTrue(mid.withinCalibratedRange());

    ShotTable.Shot quarter = t.lookup(78.0); // 25% from 72 toward 96
    assertEquals(0.225, quarter.hoodServoPosition(), 1e-9);
    assertEquals(1025.0, quarter.rpm(), 1e-9);
  }

  /**
   * Outside the measured range the table clamps and flags rather than extrapolating. Extrapolation
   * is exactly the guess this class exists to avoid: nothing was ever observed out there.
   */
  @Test
  public void testOutsideCalibratedRangeClampsAndFlags() {
    ShotTable t = table();

    ShotTable.Shot tooClose = t.lookup(20.0);
    assertEquals("clamps to the nearest row", 0.100, tooClose.hoodServoPosition(), 1e-9);
    assertEquals(900.0, tooClose.rpm(), 1e-9);
    assertFalse("below the table must not read as calibrated", tooClose.withinCalibratedRange());

    ShotTable.Shot tooFar = t.lookup(200.0);
    assertEquals(0.400, tooFar.hoodServoPosition(), 1e-9);
    assertEquals(1200.0, tooFar.rpm(), 1e-9);
    assertFalse("beyond the table must not read as calibrated", tooFar.withinCalibratedRange());
  }

  @Test
  public void testInterpolationIsMonotonicWhenTableIs() {
    ShotTable t = table();
    double prevHood = -1;
    double prevRpm = -1;
    for (double d = 48.0; d <= 120.0; d += 1.0) {
      ShotTable.Shot s = t.lookup(d);
      assertTrue(
          "hood must not decrease across a monotonic table", s.hoodServoPosition() >= prevHood);
      assertTrue("rpm must not decrease across a monotonic table", s.rpm() >= prevRpm);
      prevHood = s.hoodServoPosition();
      prevRpm = s.rpm();
    }
  }

  @Test
  public void testMalformedTablesAreRejectedLoudly() {
    // A silently mis-parsed table would command arbitrary hood and flywheel values every shot.
    assertThrows(IllegalStateException.class, () -> ShotTable.fromPoints(null));
    assertThrows(IllegalStateException.class, () -> ShotTable.fromPoints(new double[] {}));
    assertThrows(
        "length not a multiple of 3",
        IllegalStateException.class,
        () -> ShotTable.fromPoints(new double[] {48.0, 0.1, 900.0, 72.0}));
    assertThrows(
        "distances must strictly increase",
        IllegalStateException.class,
        () -> ShotTable.fromPoints(new double[] {72.0, 0.2, 1000.0, 48.0, 0.1, 900.0}));
    assertThrows(
        "duplicate distances would divide by zero when interpolating",
        IllegalStateException.class,
        () -> ShotTable.fromPoints(new double[] {48.0, 0.1, 900.0, 48.0, 0.2, 1000.0}));
  }

  @Test
  public void testSinglePointTableIsUsableAndAlwaysClamps() {
    ShotTable t = ShotTable.fromPoints(new double[] {72.0, 0.25, 1050.0});
    assertEquals(1, t.size());
    assertEquals(0.25, t.lookup(72.0).hoodServoPosition(), 1e-9);
    assertEquals(0.25, t.lookup(30.0).hoodServoPosition(), 1e-9);
    assertEquals(1050.0, t.lookup(200.0).rpm(), 1e-9);
    assertFalse(t.lookup(200.0).withinCalibratedRange());
  }

  @Test
  public void testRangeAccessors() {
    ShotTable t = table();
    assertEquals(48.0, t.minDistance(), 1e-9);
    assertEquals(120.0, t.maxDistance(), 1e-9);
    assertEquals(4, t.size());
  }
}
