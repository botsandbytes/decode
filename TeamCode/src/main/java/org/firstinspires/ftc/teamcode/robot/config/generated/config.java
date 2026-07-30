// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY
package org.firstinspires.ftc.teamcode.robot.config.generated;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;

@SuppressWarnings("unused")
public final class config {
  private config() {}

  public static Turret turret;
  public static Vision vision;
  public static Sentinel sentinel;
  public static Casablanca casablanca;
  public static Shooter shooter;
  public static Auto auto;
  public static Teleop teleop;

  public static double TURRET_MAX_TURN_POWER;
  public static double TURRET_MIN_TURN_POWER;
  public static double TURRET_MAX_POWER_OUTPUT;
  public static double TURRET_FEED_FORWARD;
  public static String TURRET_ORIENTATION_LOGO;
  public static String TURRET_ORIENTATION_USB;
  public static double TURRET_TURN_OFFSET_CONST;
  public static double TURRET_TURN_LIMIT_CONST;
  public static double TURRET_PIDF_P;
  public static double TURRET_PIDF_I;
  public static double TURRET_PIDF_D;
  public static double TURRET_PIDF_F;
  public static double TURRET_TOLERANCE_MAX_DRIFT;
  public static double TURRET_TOLERANCE_NEAR_CUTOFF;
  public static double TURRET_TOLERANCE_NEAR_VAL;
  public static double TURRET_TOLERANCE_MIN_DEG;
  public static double TURRET_TOLERANCE_MAX_DEG;
  public static boolean TURRET_ANALOG_ENCODER_ENABLED;
  public static double TURRET_ANALOG_ENCODER_ZERO_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_DEGREES_PER_VOLT;
  public static double TURRET_ANALOG_ENCODER_MIN_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_MAX_VOLTAGE;
  public static boolean TURRET_ANALOG_ENCODER_INVERTED;
  public static double VISION_CAMERA_POSITION_X;
  public static double VISION_CAMERA_POSITION_Y;
  public static double VISION_CAMERA_POSITION_Z;
  public static double VISION_CAMERA_ORIENTATION_YAW;
  public static double VISION_CAMERA_ORIENTATION_PITCH;
  public static double VISION_CAMERA_ORIENTATION_ROLL;
  public static double SENTINEL_ROBOT_WIDTH;
  public static double SENTINEL_ROTATION_LOOKAHEAD;
  public static double SENTINEL_GOALS_SIZE;
  public static double SENTINEL_GOALS_MIN_Y;
  public static double SENTINEL_GOALS_RED_GOAL_X;
  public static double SENTINEL_GOALS_RED_GOAL_Y;
  public static double SENTINEL_GOALS_BLUE_GOAL_X;
  public static double SENTINEL_GOALS_BLUE_GOAL_Y;
  public static double CASABLANCA_REPULSION_POWER;
  public static double CASABLANCA_REPULSION_DECEL_SAFETY_FACTOR;
  public static double CASABLANCA_DEPTH_SLOW_DOWN;
  public static double CASABLANCA_DEPTH_HARD_STOP;
  public static double CASABLANCA_SIDE_SLOW_DOWN;
  public static double CASABLANCA_SIDE_HARD_STOP;
  public static double CASABLANCA_LANE_BLEND_DISTANCE;
  public static double CASABLANCA_FRICTION_X;
  public static double CASABLANCA_FRICTION_Y;
  public static double CASABLANCA_FRICTION_ROT;
  public static double CASABLANCA_SMOOTHING_TIME;
  public static double CASABLANCA_SMOOTHING_BACK_LIFT_MULTIPLIER;
  public static boolean CASABLANCA_HEADING_LOCK_ENABLED;
  public static double CASABLANCA_HEADING_LOCK_KP;
  public static double CASABLANCA_HEADING_LOCK_MAX_POWER;
  public static double CASABLANCA_HEADING_LOCK_DEADBAND;
  public static double SHOOTER_MAX_RPM;
  public static boolean SHOOTER_AUTO_SHOOT_MODE;
  public static double SHOOTER_MIN_TRANSFER_THRESHOLD;
  public static double SHOOTER_HOOD_LOW_POSITION;
  public static double SHOOTER_HOOD_HIGH_POSITION;
  public static double SHOOTER_PIDF_P;
  public static double SHOOTER_PIDF_I;
  public static double SHOOTER_PIDF_D;
  public static double SHOOTER_PIDF_F;
  public static double SHOOTER_LAUNCH_PARAMS_THRESHOLD_DISTANCE;
  public static double SHOOTER_LAUNCH_PARAMS_NEAR_LAUNCH_POWER;
  public static double SHOOTER_LAUNCH_PARAMS_NEAR_WAIT_TIME;
  public static double SHOOTER_LAUNCH_PARAMS_FAR_BLUE_BASE_POWER;
  public static double SHOOTER_LAUNCH_PARAMS_FAR_RED_BASE_POWER;
  public static double SHOOTER_LAUNCH_PARAMS_FAR_POWER_SCALE;
  public static double SHOOTER_LAUNCH_PARAMS_FAR_WAIT_TIME_SCALE;
  public static int AUTO_DRINK_WAIT_MS;
  public static int AUTO_SHOOT_WAIT_MS;
  public static double AUTO_LAUNCH_POWER;
  public static double AUTO_TRANSFER_POWER;
  public static double AUTO_MIN_TRANSFER_THRESHOLD;
  public static int AUTO_OPPOSITE_DRINK_WAIT_MS;
  public static int AUTO_OPPOSITE_SHOOT_WAIT_MS;
  public static double AUTO_OPPOSITE_LAUNCH_POWER;
  public static double AUTO_OPPOSITE_TRANSFER_POWER;
  public static double AUTO_OPPOSITE_MIN_TRANSFER_THRESHOLD;
  public static double TELEOP_MAX_SPEED;
  public static Pose TELEOP_POSES_RED_START;
  public static Pose TELEOP_POSES_RED_SCORE;
  public static Pose TELEOP_POSES_RED_DRINK;
  public static Pose TELEOP_POSES_RED_PARK;
  public static Pose TELEOP_POSES_BLUE_START;
  public static Pose TELEOP_POSES_BLUE_SCORE;
  public static Pose TELEOP_POSES_BLUE_DRINK;
  public static Pose TELEOP_POSES_BLUE_PARK;

  static {
    reload();
  }

  public static synchronized void reload() {
    ConfigLoader.reload();
    turret = ConfigLoader.load(Turret.class, "turret");
    vision = ConfigLoader.load(Vision.class, "vision");
    sentinel = ConfigLoader.load(Sentinel.class, "sentinel");
    casablanca = ConfigLoader.load(Casablanca.class, "casablanca");
    shooter = ConfigLoader.load(Shooter.class, "shooter");
    auto = ConfigLoader.load(Auto.class, "auto");
    teleop = ConfigLoader.load(Teleop.class, "teleop");
    TURRET_MAX_TURN_POWER = turret.max_turn_power;
    TURRET_MIN_TURN_POWER = turret.min_turn_power;
    TURRET_MAX_POWER_OUTPUT = turret.max_power_output;
    TURRET_FEED_FORWARD = turret.feed_forward;
    TURRET_ORIENTATION_LOGO = turret.orientation.logo;
    TURRET_ORIENTATION_USB = turret.orientation.usb;
    TURRET_TURN_OFFSET_CONST = turret.turn.offset_const;
    TURRET_TURN_LIMIT_CONST = turret.turn.limit_const;
    TURRET_PIDF_P = turret.pidf.p;
    TURRET_PIDF_I = turret.pidf.i;
    TURRET_PIDF_D = turret.pidf.d;
    TURRET_PIDF_F = turret.pidf.f;
    TURRET_TOLERANCE_MAX_DRIFT = turret.tolerance.max_drift;
    TURRET_TOLERANCE_NEAR_CUTOFF = turret.tolerance.near_cutoff;
    TURRET_TOLERANCE_NEAR_VAL = turret.tolerance.near_val;
    TURRET_TOLERANCE_MIN_DEG = turret.tolerance.min_deg;
    TURRET_TOLERANCE_MAX_DEG = turret.tolerance.max_deg;
    TURRET_ANALOG_ENCODER_ENABLED = turret.analog_encoder.enabled;
    TURRET_ANALOG_ENCODER_ZERO_VOLTAGE = turret.analog_encoder.zero_voltage;
    TURRET_ANALOG_ENCODER_DEGREES_PER_VOLT = turret.analog_encoder.degrees_per_volt;
    TURRET_ANALOG_ENCODER_MIN_VOLTAGE = turret.analog_encoder.min_voltage;
    TURRET_ANALOG_ENCODER_MAX_VOLTAGE = turret.analog_encoder.max_voltage;
    TURRET_ANALOG_ENCODER_INVERTED = turret.analog_encoder.inverted;
    VISION_CAMERA_POSITION_X = vision.camera_position.x;
    VISION_CAMERA_POSITION_Y = vision.camera_position.y;
    VISION_CAMERA_POSITION_Z = vision.camera_position.z;
    VISION_CAMERA_ORIENTATION_YAW = vision.camera_orientation.yaw;
    VISION_CAMERA_ORIENTATION_PITCH = vision.camera_orientation.pitch;
    VISION_CAMERA_ORIENTATION_ROLL = vision.camera_orientation.roll;
    SENTINEL_ROBOT_WIDTH = sentinel.robot_width;
    SENTINEL_ROTATION_LOOKAHEAD = sentinel.rotation_lookahead;
    SENTINEL_GOALS_SIZE = sentinel.goals.size;
    SENTINEL_GOALS_MIN_Y = sentinel.goals.min_y;
    SENTINEL_GOALS_RED_GOAL_X = sentinel.goals.red_goal_x;
    SENTINEL_GOALS_RED_GOAL_Y = sentinel.goals.red_goal_y;
    SENTINEL_GOALS_BLUE_GOAL_X = sentinel.goals.blue_goal_x;
    SENTINEL_GOALS_BLUE_GOAL_Y = sentinel.goals.blue_goal_y;
    CASABLANCA_REPULSION_POWER = casablanca.repulsion.power;
    CASABLANCA_REPULSION_DECEL_SAFETY_FACTOR = casablanca.repulsion.decel_safety_factor;
    CASABLANCA_DEPTH_SLOW_DOWN = casablanca.depth.slow_down;
    CASABLANCA_DEPTH_HARD_STOP = casablanca.depth.hard_stop;
    CASABLANCA_SIDE_SLOW_DOWN = casablanca.side.slow_down;
    CASABLANCA_SIDE_HARD_STOP = casablanca.side.hard_stop;
    CASABLANCA_LANE_BLEND_DISTANCE = casablanca.lane_blend_distance;
    CASABLANCA_FRICTION_X = casablanca.friction.x;
    CASABLANCA_FRICTION_Y = casablanca.friction.y;
    CASABLANCA_FRICTION_ROT = casablanca.friction.rot;
    CASABLANCA_SMOOTHING_TIME = casablanca.smoothing.time;
    CASABLANCA_SMOOTHING_BACK_LIFT_MULTIPLIER = casablanca.smoothing.back_lift_multiplier;
    CASABLANCA_HEADING_LOCK_ENABLED = casablanca.heading_lock.enabled;
    CASABLANCA_HEADING_LOCK_KP = casablanca.heading_lock.kp;
    CASABLANCA_HEADING_LOCK_MAX_POWER = casablanca.heading_lock.max_power;
    CASABLANCA_HEADING_LOCK_DEADBAND = casablanca.heading_lock.deadband;
    SHOOTER_MAX_RPM = shooter.max_rpm;
    SHOOTER_AUTO_SHOOT_MODE = shooter.auto_shoot_mode;
    SHOOTER_MIN_TRANSFER_THRESHOLD = shooter.min_transfer_threshold;
    SHOOTER_HOOD_LOW_POSITION = shooter.hood.low_position;
    SHOOTER_HOOD_HIGH_POSITION = shooter.hood.high_position;
    SHOOTER_PIDF_P = shooter.pidf.p;
    SHOOTER_PIDF_I = shooter.pidf.i;
    SHOOTER_PIDF_D = shooter.pidf.d;
    SHOOTER_PIDF_F = shooter.pidf.f;
    SHOOTER_LAUNCH_PARAMS_THRESHOLD_DISTANCE = shooter.launch_params.threshold_distance;
    SHOOTER_LAUNCH_PARAMS_NEAR_LAUNCH_POWER = shooter.launch_params.near.launch_power;
    SHOOTER_LAUNCH_PARAMS_NEAR_WAIT_TIME = shooter.launch_params.near.wait_time;
    SHOOTER_LAUNCH_PARAMS_FAR_BLUE_BASE_POWER = shooter.launch_params.far.blue_base_power;
    SHOOTER_LAUNCH_PARAMS_FAR_RED_BASE_POWER = shooter.launch_params.far.red_base_power;
    SHOOTER_LAUNCH_PARAMS_FAR_POWER_SCALE = shooter.launch_params.far.power_scale;
    SHOOTER_LAUNCH_PARAMS_FAR_WAIT_TIME_SCALE = shooter.launch_params.far.wait_time_scale;
    AUTO_DRINK_WAIT_MS = auto.drink_wait_ms;
    AUTO_SHOOT_WAIT_MS = auto.shoot_wait_ms;
    AUTO_LAUNCH_POWER = auto.launch_power;
    AUTO_TRANSFER_POWER = auto.transfer_power;
    AUTO_MIN_TRANSFER_THRESHOLD = auto.min_transfer_threshold;
    AUTO_OPPOSITE_DRINK_WAIT_MS = auto.opposite_drink_wait_ms;
    AUTO_OPPOSITE_SHOOT_WAIT_MS = auto.opposite_shoot_wait_ms;
    AUTO_OPPOSITE_LAUNCH_POWER = auto.opposite_launch_power;
    AUTO_OPPOSITE_TRANSFER_POWER = auto.opposite_transfer_power;
    AUTO_OPPOSITE_MIN_TRANSFER_THRESHOLD = auto.opposite_min_transfer_threshold;
    TELEOP_MAX_SPEED = teleop.max_speed;
    TELEOP_POSES_RED_START = teleop.poses.red.start;
    TELEOP_POSES_RED_SCORE = teleop.poses.red.score;
    TELEOP_POSES_RED_DRINK = teleop.poses.red.drink;
    TELEOP_POSES_RED_PARK = teleop.poses.red.park;
    TELEOP_POSES_BLUE_START = teleop.poses.blue.start;
    TELEOP_POSES_BLUE_SCORE = teleop.poses.blue.score;
    TELEOP_POSES_BLUE_DRINK = teleop.poses.blue.drink;
    TELEOP_POSES_BLUE_PARK = teleop.poses.blue.park;
  }

  public static final class Turret {
    /**
     * Upper speed limit (0.0 to 1.0) allowed for chassis turn adjustments when auto-aligning to the
     * goal. Minimum: 0.0 Maximum: 1.0
     */
    public double max_turn_power;

    /**
     * Minimum power (0.0 to 1.0) sent to the drivetrain during auto-turn corrections. Overcomes
     * static wheel friction (stiction). If the robot stops turning before aligning, increase this.
     * Minimum: 0.0 Maximum: 1.0
     */
    public double min_turn_power;

    /**
     * Maximum power limit (0.0 to 1.0) commanded to the turret CRServo. Limits maximum turret
     * rotation speed. Minimum: 0.0 Maximum: 1.0
     */
    public double max_power_output;

    /**
     * Feedforward power component (0.0 to 1.0) added to turret rotation to overcome gear backlash
     * and friction. Applied in the direction of movement. Minimum: 0.0 Maximum: 1.0
     */
    public double feed_forward;

    public static final class Orientation {
      /**
       * Mounting orientation direction of the hub's logo panel (e.g. RIGHT, LEFT, UP, DOWN,
       * FORWARD, BACKWARD).
       */
      public String logo;

      /** Mounting direction of the hub's USB ports. */
      public String usb;
    }

    public Orientation orientation;

    public static final class Turn {
      /**
       * Encoder-to-angle mapping offset (degrees) to align the physical turret zero orientation
       * with the chassis.
       */
      public double offset_const;

      /**
       * Mechanical rotation boundary (degrees) measured from center. Prevents the turret from
       * rotating past this limit and tearing internal wires.
       */
      public double limit_const;
    }

    public Turn turn;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    public static final class Tolerance {
      /**
       * Maximum allowed chassis drift (inches) from target pose before auto-corrective drive
       * motions engage. Minimum: 0.0
       */
      public double max_drift;

      /**
       * Distance boundary (inches) below which the target is considered close, switching to static
       * tolerance. Minimum: 0.0
       */
      public double near_cutoff;

      /** Target alignment tolerance (degrees) when within the near cutoff range. Minimum: 0.0 */
      public double near_val;

      /**
       * Minimum dynamic alignment tolerance (degrees) allowed. Prevents the turret from hunting
       * indefinitely when very close. Minimum: 0.0
       */
      public double min_deg;

      /**
       * Maximum dynamic alignment tolerance (degrees) allowed. Prevents launching if the turret
       * orientation error exceeds this. Minimum: 0.0
       */
      public double max_deg;
    }

    public Tolerance tolerance;

    public static final class AnalogEncoder {
      /**
       * Enable/disable automatic heading lock when the driver releases the turn joystick in TeleOp.
       */
      public boolean enabled;

      /**
       * Analog voltage (V) corresponding to 0 degree turret angle position. Minimum: 0.0 Maximum:
       * 3.3
       */
      public double zero_voltage;

      /**
       * Scale factor (degrees/volt) converting analog input voltage delta to turret angle in
       * degrees.
       */
      public double degrees_per_volt;

      /** Minimum safe voltage limit for the analog encoder. Minimum: 0.0 Maximum: 3.3 */
      public double min_voltage;

      /** Maximum safe voltage limit for the analog encoder. Minimum: 0.0 Maximum: 3.3 */
      public double max_voltage;

      /** True if increasing turret angle produces decreasing analog voltage. */
      public boolean inverted;
    }

    public AnalogEncoder analog_encoder;
  }

  public static final class Vision {
    public static final class CameraPosition {
      /** X value coordinate. */
      public double x;

      /** Y value coordinate. */
      public double y;

      public double z;
    }

    public CameraPosition camera_position;

    public static final class CameraOrientation {
      public double yaw;
      public double pitch;
      public double roll;
    }

    public CameraOrientation camera_orientation;
  }

  public static final class Sentinel {
    /**
     * Total physical width of the robot (inches) used to build the collision footprint. Determines
     * how close the robot can get to walls or goals. Minimum: 0.0
     */
    public double robot_width;

    /**
     * Angle (radians) used to project the robot's future rotation. If the robot would violate a
     * goal zone at this lookahead angle, it prevents the turn. Minimum: 0.0
     */
    public double rotation_lookahead;

    public static final class Goals {
      /** Length (inches) of one side of the square scoring goal zones on the field. Minimum: 0.0 */
      public double size;

      /**
       * The Y-coordinate boundary (inches) on the field where the scoring zones begin. Used to
       * calculate red/blue alliance goal boxes. Minimum: 0.0
       */
      public double min_y;

      /** Goal center X coordinate for RED alliance. Minimum: 0.0 */
      public double red_goal_x;

      /** Goal center Y coordinate for RED alliance. Minimum: 0.0 */
      public double red_goal_y;

      /** Goal center X coordinate for BLUE alliance. Minimum: 0.0 */
      public double blue_goal_x;

      /** Goal center Y coordinate for BLUE alliance. Minimum: 0.0 */
      public double blue_goal_y;
    }

    public Goals goals;
  }

  public static final class Casablanca {
    public static final class Repulsion {
      /**
       * Motor power (0.0 to 1.0) commanded to push the robot away when it violates a goal zone
       * boundary. Minimum: 0.0 Maximum: 1.0
       */
      public double power;

      /**
       * Scale factor (0.0 to 1.0) for predictive braking. Lowering this makes braking engage
       * sooner/more aggressively to guarantee the robot doesn't enter the goal zone. Minimum: 0.0
       * Maximum: 1.0
       */
      public double decel_safety_factor;
    }

    public Repulsion repulsion;

    public static final class Depth {
      /**
       * Distance (inches) from the goal zone at which speed reduction warning starts. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from the goal zone where drive power towards the zone is cut to zero.
       * Minimum: 0.0
       */
      public double hard_stop;
    }

    public Depth depth;

    public static final class Side {
      /**
       * Distance (inches) from the goal zone at which speed reduction warning starts. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from the goal zone where drive power towards the zone is cut to zero.
       * Minimum: 0.0
       */
      public double hard_stop;
    }

    public Side side;

    /**
     * Distance (inches) over which safety slows downs are interpolated from 0% to 100% strength to
     * prevent jerky speed changes. Minimum: 0.0
     */
    public double lane_blend_distance;

    public static final class Friction {
      /** X value coordinate. */
      public double x;

      /** Y value coordinate. */
      public double y;

      /**
       * Minimum turn/yaw drive power (0.0 to 1.0) needed to overcome static wheel stiction.
       * Calibrated using Friction Calibration OpMode. Minimum: 0.0 Maximum: 1.0
       */
      public double rot;
    }

    public Friction friction;

    public static final class Smoothing {
      /**
       * Joystick input filter time constant (seconds). Higher values make acceleration and
       * deceleration smoother but increase input lag. Minimum: 0.0
       */
      public double time;

      /**
       * Divisor scaling factor for deceleration. Divides the deceleration rate when stopping to
       * slow down the deceleration curve, preventing the robot from tipping forward and lifting its
       * back wheels. Minimum: 0.0
       */
      public double back_lift_multiplier;
    }

    public Smoothing smoothing;

    public static final class HeadingLock {
      /**
       * Enable/disable automatic heading lock when the driver releases the turn joystick in TeleOp.
       */
      public boolean enabled;

      /**
       * Proportional gain for the controller loop. Higher values correct errors more aggressively.
       * Minimum: 0.0
       */
      public double kp;

      /**
       * Maximum motor power (0.0 to 1.0) the loop is allowed to command. Clamped to be completely
       * imperceptible. Minimum: 0.0 Maximum: 1.0
       */
      public double max_power;

      /**
       * Joystick threshold (0.0 to 1.0) below which the heading lock engages. Minimum: 0.0 Maximum:
       * 1.0
       */
      public double deadband;
    }

    public HeadingLock heading_lock;
  }

  public static final class Shooter {
    /**
     * Maximum rotational speed (RPM) of the motor. Used to scale power percentage to velocity.
     * Minimum: 0.0
     */
    public double max_rpm;

    public boolean auto_shoot_mode;

    /**
     * Flywheel speed threshold (percentage, 0.0 to 1.0) required to trigger the transfer belt.
     * Prevents launching rings before the shooter is up to speed. Minimum: 0.0 Maximum: 1.0
     */
    public double min_transfer_threshold;

    public static final class Hood {
      /**
       * Hood servo low/retracted position coefficient (0.0 to 1.0) for short-range shots. Minimum:
       * 0.0 Maximum: 1.0
       */
      public double low_position;

      /**
       * Hood servo high/extended position coefficient (0.0 to 1.0) for long-range shots. Minimum:
       * 0.0 Maximum: 1.0
       */
      public double high_position;
    }

    public Hood hood;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    public static final class LaunchParams {
      /**
       * Distance boundary (inches) separating close-range shots from long-range shots. Minimum: 0.0
       */
      public double threshold_distance;

      public static final class Near {
        /**
         * Flywheel power ratio (0.0 to 1.0) used for close-range shots. Minimum: 0.0 Maximum: 1.0
         */
        public double launch_power;

        /**
         * Delay (milliseconds) to keep the flywheel running after firing a near shot. Minimum: 0.0
         */
        public double wait_time;
      }

      public Near near;

      public static final class Far {
        /**
         * Base flywheel power ratio (0.0 to 1.0) for Blue alliance shots beyond the threshold
         * distance. Minimum: 0.0 Maximum: 1.0
         */
        public double blue_base_power;

        /**
         * Base flywheel power ratio (0.0 to 1.0) for Red alliance shots beyond the threshold
         * distance. Red shoots from farther away so this value is intentionally higher. Minimum:
         * 0.0 Maximum: 1.0
         */
        public double red_base_power;

        /**
         * Distance divisor (inches) to scale up flywheel power as distance increases (power =
         * base_power + extra_distance / power_scale). Minimum: 0.0
         */
        public double power_scale;

        /**
         * Multiplier converting distance to run-time (wait_time = distance * wait_time_scale) in
         * milliseconds. Minimum: 0.0
         */
        public double wait_time_scale;
      }

      public Far far;
    }

    public LaunchParams launch_params;
  }

  public static final class Auto {
    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. Minimum: 0.0 */
    public int drink_wait_ms;

    /**
     * Wait time (milliseconds) allowed to fire a shot before ending the shoot command. Minimum: 0.0
     */
    public int shoot_wait_ms;

    /** Flywheel power ratio (0.0 to 1.0) used for close-range shots. Minimum: 0.0 Maximum: 1.0 */
    public double launch_power;

    /**
     * Target power coefficient (0.0 to 1.0) for autonomous transfer motor. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double transfer_power;

    /**
     * Flywheel speed threshold (percentage, 0.0 to 1.0) required to trigger the transfer belt.
     * Prevents launching rings before the shooter is up to speed. Minimum: 0.0 Maximum: 1.0
     */
    public double min_transfer_threshold;

    /**
     * Wait delay (milliseconds) at the drink gate for opposite alliance autonomous. Minimum: 0.0
     */
    public int opposite_drink_wait_ms;

    /**
     * Wait time (milliseconds) allowed to fire a shot for opposite alliance autonomous. Minimum:
     * 0.0
     */
    public int opposite_shoot_wait_ms;

    /**
     * Target power coefficient (0.0 to 1.0) for opposite alliance autonomous launcher. Minimum: 0.0
     * Maximum: 1.0
     */
    public double opposite_launch_power;

    /**
     * Target power coefficient (0.0 to 1.0) for opposite alliance autonomous transfer. Minimum: 0.0
     * Maximum: 1.0
     */
    public double opposite_transfer_power;

    /**
     * Shooter minimum transfer threshold velocity ratio for opposite auto. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double opposite_min_transfer_threshold;
  }

  public static final class Teleop {
    /**
     * Maximum speed scaler (0.0 to 1.0) applied to chassis manual TeleOp drive inputs. Minimum: 0.0
     * Maximum: 1.0
     */
    public double max_speed;

    public static final class Poses {
      public static final class Red {
        /** Starting position coordinate pose. */
        public Pose start;

        /** High goal target scoring position coordinate pose. */
        public Pose score;

        /** Drink gate alignment coordinate pose. */
        public Pose drink;

        /** Parking destination coordinate pose. */
        public Pose park;
      }

      public Red red;

      public static final class Blue {
        /** Starting position coordinate pose. */
        public Pose start;

        /** High goal target scoring position coordinate pose. */
        public Pose score;

        /** Drink gate alignment coordinate pose. */
        public Pose drink;

        /** Parking destination coordinate pose. */
        public Pose park;
      }

      public Blue blue;
    }

    public Poses poses;
  }

  public static final class NormalAuto {
    /** Control point pose on path to drink gate. */
    public Pose drinkCp;

    /** Ending position coordinate pose at the drink gate. */
    public Pose drinkEnd;

    /** Final scoring target position coordinate pose. */
    public Pose finalScore;

    /** Control point pose on path to first sample pickup. */
    public Pose pickup1Cp;

    /** Ending position coordinate pose for first sample pickup. */
    public Pose pickup1End;

    /** Control point pose on path to second sample pickup. */
    public Pose pickup2Cp;

    /** Ending position coordinate pose for second sample pickup. */
    public Pose pickup2End;

    /** Control point pose on path to third sample pickup. */
    public Pose pickup3Cp;

    /** Ending position coordinate pose for third sample pickup. */
    public Pose pickup3End;

    /** High goal target scoring position coordinate pose. */
    public Pose score;

    /** Starting position coordinate pose. */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. */
    public int drinkWaitMs;

    /** Wait time (milliseconds) allowed to fire a shot before ending the shoot command. */
    public int shootWaitMs;

    /** Flywheel power ratio (0.0 to 1.0) used for close-range shots. */
    public double launchPower;

    /** Target power coefficient (0.0 to 1.0) for autonomous transfer motor. */
    public double transferPower;

    /**
     * Flywheel speed threshold (percentage, 0.0 to 1.0) required to trigger the transfer belt.
     * Prevents launching rings before the shooter is up to speed.
     */
    public double minTransferThreshold;

    /** Wait delay (milliseconds) at the drink gate for opposite alliance autonomous. */
    public int oppositeDrinkWaitMs;

    /** Wait time (milliseconds) allowed to fire a shot for opposite alliance autonomous. */
    public int oppositeShootWaitMs;

    /** Target power coefficient (0.0 to 1.0) for opposite alliance autonomous launcher. */
    public double oppositeLaunchPower;

    /** Target power coefficient (0.0 to 1.0) for opposite alliance autonomous transfer. */
    public double oppositeTransferPower;

    /** Shooter minimum transfer threshold velocity ratio for opposite auto. */
    public double oppositeMinTransferThreshold;
  }

  public static final class OppositeAuto {
    /** Control point pose on path to drink gate. */
    public Pose drinkCp;

    /** Ending position coordinate pose at the drink gate. */
    public Pose drinkEnd;

    /** Parking destination coordinate pose. */
    public Pose park;

    /** Control point pose on path to second sample pickup. */
    public Pose pickup2Cp;

    /** Ending position coordinate pose for second sample pickup. */
    public Pose pickup2End;

    /** Control point pose on path to third sample pickup. */
    public Pose pickup3Cp;

    /** Ending position coordinate pose for third sample pickup. */
    public Pose pickup3End;

    /** Control point pose on path to fourth sample pickup. */
    public Pose pickup4Cp;

    /** Ending position coordinate pose for fourth sample pickup. */
    public Pose pickup4End;

    /** High goal target scoring position coordinate pose. */
    public Pose score;

    /** Starting position coordinate pose. */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. */
    public int drinkWaitMs;

    /** Wait time (milliseconds) allowed to fire a shot before ending the shoot command. */
    public int shootWaitMs;

    /** Flywheel power ratio (0.0 to 1.0) used for close-range shots. */
    public double launchPower;

    /** Target power coefficient (0.0 to 1.0) for autonomous transfer motor. */
    public double transferPower;

    /**
     * Flywheel speed threshold (percentage, 0.0 to 1.0) required to trigger the transfer belt.
     * Prevents launching rings before the shooter is up to speed.
     */
    public double minTransferThreshold;
  }

  public static MatchProfile loadMatchProfile(Alliance alliance) {
    return MatchProfile.loadMatchProfile(alliance);
  }
}
