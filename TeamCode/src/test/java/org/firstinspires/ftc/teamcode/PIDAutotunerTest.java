package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.utilities.PIDAutotuner;
import org.junit.Before;
import org.junit.Test;

public class PIDAutotunerTest {

  private PIDAutotuner autotuner;

  @Before
  public void setUp() {
    autotuner = new PIDAutotuner();
  }

  @Test
  public void testInitialState() {
    assertFalse(autotuner.isRunning());
    assertFalse(autotuner.isComplete());
    assertFalse(autotuner.isFailed());
    assertEquals(0, autotuner.getCrossingCount());
  }

  @Test
  public void testStartAutotune() {
    autotuner.startAutotune(0.0, 0.0, 0.4);
    assertTrue(autotuner.isRunning());
    assertFalse(autotuner.isComplete());
    assertFalse(autotuner.isFailed());
  }

  @Test
  public void testCancel() {
    autotuner.startAutotune(0.0, 0.0, 0.4);
    assertTrue(autotuner.isRunning());
    autotuner.cancel();
    assertFalse(autotuner.isRunning());
  }

  @Test
  public void testOutputPowerDirection() {
    autotuner.startAutotune(0.0, 0.0, 0.4);
    // When currentVal is negative (-5.0), error is +5.0 -> output should be +0.4
    double power1 = autotuner.updateAutotune(-5.0);
    assertEquals(0.4, power1, 1e-4);

    // When currentVal is positive (+5.0), error is -5.0 -> output should be -0.4
    double power2 = autotuner.updateAutotune(5.0);
    assertEquals(-0.4, power2, 1e-4);
  }

  @Test
  public void testNoiseDebouncePreventsInstantCompletion() {
    autotuner.startAutotune(0.0, 0.0, 0.4);

    // Rapid noise fluctuations near zero in consecutive loop calls
    autotuner.updateAutotune(0.01);
    autotuner.updateAutotune(-0.01);
    autotuner.updateAutotune(0.01);
    autotuner.updateAutotune(-0.01);
    autotuner.updateAutotune(0.01);
    autotuner.updateAutotune(-0.01);

    // Noise within 120ms should NOT count as valid oscillation crossings
    assertTrue(
        "High frequency noise should not complete autotuner", autotuner.getCrossingCount() <= 1);
    assertTrue("Autotuner should still be running", autotuner.isRunning());
  }
}
