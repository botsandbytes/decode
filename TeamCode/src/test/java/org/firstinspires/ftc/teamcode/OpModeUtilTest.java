package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.junit.Test;

public class OpModeUtilTest {

  @Test
  public void testBlackboardPoseKeys() {
    assertEquals("RED_POSE", OpModeUtil.getBlackboardPoseKey(Alliance.RED));
    assertEquals("BLUE_POSE", OpModeUtil.getBlackboardPoseKey(Alliance.BLUE));
  }
}
