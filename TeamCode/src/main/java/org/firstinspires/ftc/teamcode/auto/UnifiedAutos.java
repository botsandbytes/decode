package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.teamcode.records.Alliance;

public final class UnifiedAutos {

  @OpModeRegistrar
  public static void register(OpModeManager manager) {
    manager.register("Blue Auto NEW", BlueAutoNew.class);
    manager.register("Red Auto NEW", RedAutoNew.class);
    manager.register("Blue Opposite NEW", BlueOppositeNew.class);
    manager.register("Red Opposite NEW", RedOppositeNew.class);
  }

  public static class BlueAutoNew extends AllianceAutoNew {
    public BlueAutoNew() {
      super(Alliance.BLUE);
    }
  }

  public static class RedAutoNew extends AllianceAutoNew {
    public RedAutoNew() {
      super(Alliance.RED);
    }
  }

  public static class BlueOppositeNew extends AllianceOppositeNew {
    public BlueOppositeNew() {
      super(Alliance.BLUE);
    }
  }

  public static class RedOppositeNew extends AllianceOppositeNew {
    public RedOppositeNew() {
      super(Alliance.RED);
    }
  }
}
