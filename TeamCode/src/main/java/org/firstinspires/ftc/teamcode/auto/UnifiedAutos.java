package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.records.Alliance;

public final class UnifiedAutos {

  @Autonomous(name = "Blue Auto NEW", group = "Autonomous")
  public static class BlueAutoNew extends AllianceAutoNew {
    public BlueAutoNew() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red Auto NEW", group = "Autonomous")
  public static class RedAutoNew extends AllianceAutoNew {
    public RedAutoNew() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue Opposite NEW", group = "Autonomous")
  public static class BlueOppositeNew extends AllianceOppositeNew {
    public BlueOppositeNew() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red Opposite NEW", group = "Autonomous")
  public static class RedOppositeNew extends AllianceOppositeNew {
    public RedOppositeNew() {
      super(Alliance.RED);
    }
  }
}
