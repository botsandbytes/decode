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

  public static boolean TURRET_ENABLED;
  public static double TURRET_MAX_POWER_OUTPUT;
  public static boolean TURRET_SERVO_DIRECTION_INVERTED;
  public static double TURRET_KS_POSITIVE;
  public static double TURRET_KS_NEGATIVE;
  public static String TURRET_ORIENTATION_LOGO;
  public static String TURRET_ORIENTATION_USB;
  public static double TURRET_TRAVEL_MIN_ANGLE;
  public static double TURRET_TRAVEL_MAX_ANGLE;
  public static boolean TURRET_STALL_ENABLED;
  public static double TURRET_STALL_POWER_THRESHOLD;
  public static double TURRET_STALL_MIN_PROGRESS_DEG;
  public static double TURRET_STALL_TIMEOUT_SEC;
  public static boolean TURRET_RUNAWAY_ENABLED;
  public static double TURRET_RUNAWAY_DIVERGENCE_DEG;
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
  public static double TURRET_ANALOG_ENCODER_FULL_SCALE_VOLTAGE;
  public static boolean TURRET_ANALOG_ENCODER_INVERTED;
  public static double VISION_CAMERA_POSITION_X;
  public static double VISION_CAMERA_POSITION_Y;
  public static double VISION_CAMERA_POSITION_Z;
  public static double VISION_CAMERA_ORIENTATION_YAW;
  public static double VISION_CAMERA_ORIENTATION_PITCH;
  public static double VISION_CAMERA_ORIENTATION_ROLL;
  public static double SENTINEL_ROBOT_WIDTH;
  public static double SENTINEL_ROTATION_LOOKAHEAD_TIME;
  public static double SENTINEL_GOALS_SIZE;
  public static double SENTINEL_GOALS_MIN_Y;
  public static double SENTINEL_GOALS_RED_GOAL_X;
  public static double SENTINEL_GOALS_RED_GOAL_Y;
  public static double SENTINEL_GOALS_BLUE_GOAL_X;
  public static double SENTINEL_GOALS_BLUE_GOAL_Y;
  public static boolean CASABLANCA_ENABLE_DEPTH_PROTECTION;
  public static boolean CASABLANCA_ENABLE_SIDE_PROTECTION;
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
  public static double CASABLANCA_HEADING_LOCK_INTENT_THRESHOLD;
  public static double CASABLANCA_HEADING_LOCK_KS_MOVING;
  public static double CASABLANCA_HEADING_LOCK_MOVING_SPEED_THRESHOLD;
  public static double CASABLANCA_HEADING_LOCK_MAX_POWER;
  public static double CASABLANCA_HEADING_LOCK_ERROR_DEADBAND_DEG;
  public static double CASABLANCA_HEADING_LOCK_SETTLE_RATE_DPS;
  public static boolean SHOOTER_USE_FTC_PID;
  public static double SHOOTER_KS;
  public static double SHOOTER_NOMINAL_VOLTAGE;
  public static double SHOOTER_MAX_VOLTAGE_COMPENSATION;
  public static double SHOOTER_VOLTAGE_REFRESH_SEC;
  public static double SHOOTER_MAX_RPM;
  public static double SHOOTER_CONSTANT_RPM;
  public static double SHOOTER_MIN_TRANSFER_THRESHOLD;
  public static double SHOOTER_MAX_VELOCITY_THRESHOLD;
  public static double SHOOTER_FEED_RELEASE_THRESHOLD;
  public static double SHOOTER_FEED_INTAKE_POWER;
  public static double SHOOTER_FEED_TRANSFER_POWER;
  public static double SHOOTER_LONG_HOOD_POWER_THRESHOLD;
  public static double SHOOTER_BALL_DETECTION_DIP_FRACTION;
  public static double SHOOTER_BALL_DETECTION_REBOUND_FRACTION;
  public static double SHOOTER_BALL_DETECTION_BASELINE_ALPHA;
  public static int SHOOTER_BALL_DETECTION_REFRACTORY_MS;
  public static double SHOOTER_PIDF_P;
  public static double SHOOTER_PIDF_I;
  public static double SHOOTER_PIDF_D;
  public static double SHOOTER_PIDF_F;
  public static double SHOOTER_MOTOR_PIDF_P;
  public static double SHOOTER_MOTOR_PIDF_I;
  public static double SHOOTER_MOTOR_PIDF_D;
  public static double SHOOTER_MOTOR_PIDF_F;
  public static double SHOOTER_INTEGRAL_BAND_TICKS;
  public static double SHOOTER_INTEGRAL_MAX_CONTRIBUTION;
  public static double SHOOTER_BALLISTICS_V0;
  public static double SHOOTER_BALLISTICS_K;
  public static double SHOOTER_BALLISTICS_MAGNUS_L;
  public static double SHOOTER_BALLISTICS_LAUNCH_HEIGHT_INCHES;
  public static double SHOOTER_BALLISTICS_GOAL_HEIGHT_INCHES;
  public static double SHOOTER_BALLISTICS_G_INCHES_PER_SEC2;
  public static double SHOOTER_BALLISTICS_MIN_HOOD_ANGLE_DEG;
  public static double SHOOTER_BALLISTICS_MAX_HOOD_ANGLE_DEG;
  public static double SHOOTER_BALLISTICS_MIN_HOOD_SERVO_POS;
  public static double SHOOTER_BALLISTICS_MAX_HOOD_SERVO_POS;
  public static double SHOOTER_BALLISTICS_MAX_MOVING_SPEED_IPS;
  public static double SHOOTER_BALLISTICS_MIN_VALID_DISTANCE;
  public static double SHOOTER_BALLISTICS_MAX_VALID_DISTANCE;
  public static double SHOOTER_BALLISTICS_LEAD_BIAS_GAIN;
  public static double SHOOTER_BALLISTICS_V0_REFERENCE_RPM;
  public static double SHOOTER_BALLISTICS_PREFERRED_SHOT_RPM;
  public static double SHOOTER_BALLISTICS_MAX_SHOT_RPM;
  public static double SHOOTER_BALLISTICS_V0_MARGIN_FRACTION;
  public static double SHOOTER_BALLISTICS_RPM_UPDATE_DEADBAND;
  public static double SHOOTER_BALLISTICS_FLIGHT_TIME_SEC_PER_INCH;
  public static double SHOOTER_BALLISTICS_LEAD_MIN_SPEED_IPS;
  public static double[] SHOOTER_SHOT_TABLE_POINTS;
  public static double[] AUTO_SHOT_TIME_POINTS;
  public static int AUTO_BALLS_PER_SHOT_COUNT;
  public static int AUTO_DRINK_WAIT_MS;
  public static int AUTO_SHOOT_WAIT_MS;
  public static double AUTO_LAUNCH_POWER;
  public static double AUTO_TRANSFER_POWER;
  public static int AUTO_OPPOSITE_DRINK_WAIT_MS;
  public static int AUTO_OPPOSITE_SHOOT_WAIT_MS;
  public static double AUTO_OPPOSITE_LAUNCH_POWER;
  public static double AUTO_OPPOSITE_TRANSFER_POWER;
  public static double TELEOP_MAX_SPEED;
  public static boolean TELEOP_FIELD_CENTRIC;
  public static double TELEOP_FIELD_CENTRIC_OFFSET_DEG;
  public static double TELEOP_INTAKE_POWER;
  public static double TELEOP_TRANSFER_POWER;
  public static double TELEOP_MANUAL_REV_POWER;
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
    TURRET_ENABLED = turret.enabled;
    TURRET_MAX_POWER_OUTPUT = turret.max_power_output;
    TURRET_SERVO_DIRECTION_INVERTED = turret.servo_direction_inverted;
    TURRET_KS_POSITIVE = turret.ks_positive;
    TURRET_KS_NEGATIVE = turret.ks_negative;
    TURRET_ORIENTATION_LOGO = turret.orientation.logo;
    TURRET_ORIENTATION_USB = turret.orientation.usb;
    TURRET_TRAVEL_MIN_ANGLE = turret.travel.min_angle;
    TURRET_TRAVEL_MAX_ANGLE = turret.travel.max_angle;
    TURRET_STALL_ENABLED = turret.stall.enabled;
    TURRET_STALL_POWER_THRESHOLD = turret.stall.power_threshold;
    TURRET_STALL_MIN_PROGRESS_DEG = turret.stall.min_progress_deg;
    TURRET_STALL_TIMEOUT_SEC = turret.stall.timeout_sec;
    TURRET_RUNAWAY_ENABLED = turret.runaway.enabled;
    TURRET_RUNAWAY_DIVERGENCE_DEG = turret.runaway.divergence_deg;
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
    TURRET_ANALOG_ENCODER_FULL_SCALE_VOLTAGE = turret.analog_encoder.full_scale_voltage;
    TURRET_ANALOG_ENCODER_INVERTED = turret.analog_encoder.inverted;
    VISION_CAMERA_POSITION_X = vision.camera_position.x;
    VISION_CAMERA_POSITION_Y = vision.camera_position.y;
    VISION_CAMERA_POSITION_Z = vision.camera_position.z;
    VISION_CAMERA_ORIENTATION_YAW = vision.camera_orientation.yaw;
    VISION_CAMERA_ORIENTATION_PITCH = vision.camera_orientation.pitch;
    VISION_CAMERA_ORIENTATION_ROLL = vision.camera_orientation.roll;
    SENTINEL_ROBOT_WIDTH = sentinel.robot_width;
    SENTINEL_ROTATION_LOOKAHEAD_TIME = sentinel.rotation_lookahead_time;
    SENTINEL_GOALS_SIZE = sentinel.goals.size;
    SENTINEL_GOALS_MIN_Y = sentinel.goals.min_y;
    SENTINEL_GOALS_RED_GOAL_X = sentinel.goals.red_goal_x;
    SENTINEL_GOALS_RED_GOAL_Y = sentinel.goals.red_goal_y;
    SENTINEL_GOALS_BLUE_GOAL_X = sentinel.goals.blue_goal_x;
    SENTINEL_GOALS_BLUE_GOAL_Y = sentinel.goals.blue_goal_y;
    CASABLANCA_ENABLE_DEPTH_PROTECTION = casablanca.enable_depth_protection;
    CASABLANCA_ENABLE_SIDE_PROTECTION = casablanca.enable_side_protection;
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
    CASABLANCA_HEADING_LOCK_INTENT_THRESHOLD = casablanca.heading_lock.intent_threshold;
    CASABLANCA_HEADING_LOCK_KS_MOVING = casablanca.heading_lock.ks_moving;
    CASABLANCA_HEADING_LOCK_MOVING_SPEED_THRESHOLD = casablanca.heading_lock.moving_speed_threshold;
    CASABLANCA_HEADING_LOCK_MAX_POWER = casablanca.heading_lock.max_power;
    CASABLANCA_HEADING_LOCK_ERROR_DEADBAND_DEG = casablanca.heading_lock.error_deadband_deg;
    CASABLANCA_HEADING_LOCK_SETTLE_RATE_DPS = casablanca.heading_lock.settle_rate_dps;
    SHOOTER_USE_FTC_PID = shooter.use_ftc_pid;
    SHOOTER_KS = shooter.ks;
    SHOOTER_NOMINAL_VOLTAGE = shooter.nominal_voltage;
    SHOOTER_MAX_VOLTAGE_COMPENSATION = shooter.max_voltage_compensation;
    SHOOTER_VOLTAGE_REFRESH_SEC = shooter.voltage_refresh_sec;
    SHOOTER_MAX_RPM = shooter.max_rpm;
    SHOOTER_CONSTANT_RPM = shooter.constant_rpm;
    SHOOTER_MIN_TRANSFER_THRESHOLD = shooter.min_transfer_threshold;
    SHOOTER_MAX_VELOCITY_THRESHOLD = shooter.max_velocity_threshold;
    SHOOTER_FEED_RELEASE_THRESHOLD = shooter.feed_release_threshold;
    SHOOTER_FEED_INTAKE_POWER = shooter.feed_intake_power;
    SHOOTER_FEED_TRANSFER_POWER = shooter.feed_transfer_power;
    SHOOTER_LONG_HOOD_POWER_THRESHOLD = shooter.long_hood_power_threshold;
    SHOOTER_BALL_DETECTION_DIP_FRACTION = shooter.ball_detection.dip_fraction;
    SHOOTER_BALL_DETECTION_REBOUND_FRACTION = shooter.ball_detection.rebound_fraction;
    SHOOTER_BALL_DETECTION_BASELINE_ALPHA = shooter.ball_detection.baseline_alpha;
    SHOOTER_BALL_DETECTION_REFRACTORY_MS = shooter.ball_detection.refractory_ms;
    SHOOTER_PIDF_P = shooter.pidf.p;
    SHOOTER_PIDF_I = shooter.pidf.i;
    SHOOTER_PIDF_D = shooter.pidf.d;
    SHOOTER_PIDF_F = shooter.pidf.f;
    SHOOTER_MOTOR_PIDF_P = shooter.motor_pidf.p;
    SHOOTER_MOTOR_PIDF_I = shooter.motor_pidf.i;
    SHOOTER_MOTOR_PIDF_D = shooter.motor_pidf.d;
    SHOOTER_MOTOR_PIDF_F = shooter.motor_pidf.f;
    SHOOTER_INTEGRAL_BAND_TICKS = shooter.integral.band_ticks;
    SHOOTER_INTEGRAL_MAX_CONTRIBUTION = shooter.integral.max_contribution;
    SHOOTER_BALLISTICS_V0 = shooter.ballistics.v0;
    SHOOTER_BALLISTICS_K = shooter.ballistics.k;
    SHOOTER_BALLISTICS_MAGNUS_L = shooter.ballistics.magnus_l;
    SHOOTER_BALLISTICS_LAUNCH_HEIGHT_INCHES = shooter.ballistics.launch_height_inches;
    SHOOTER_BALLISTICS_GOAL_HEIGHT_INCHES = shooter.ballistics.goal_height_inches;
    SHOOTER_BALLISTICS_G_INCHES_PER_SEC2 = shooter.ballistics.g_inches_per_sec2;
    SHOOTER_BALLISTICS_MIN_HOOD_ANGLE_DEG = shooter.ballistics.min_hood_angle_deg;
    SHOOTER_BALLISTICS_MAX_HOOD_ANGLE_DEG = shooter.ballistics.max_hood_angle_deg;
    SHOOTER_BALLISTICS_MIN_HOOD_SERVO_POS = shooter.ballistics.min_hood_servo_pos;
    SHOOTER_BALLISTICS_MAX_HOOD_SERVO_POS = shooter.ballistics.max_hood_servo_pos;
    SHOOTER_BALLISTICS_MAX_MOVING_SPEED_IPS = shooter.ballistics.max_moving_speed_ips;
    SHOOTER_BALLISTICS_MIN_VALID_DISTANCE = shooter.ballistics.min_valid_distance;
    SHOOTER_BALLISTICS_MAX_VALID_DISTANCE = shooter.ballistics.max_valid_distance;
    SHOOTER_BALLISTICS_LEAD_BIAS_GAIN = shooter.ballistics.lead_bias_gain;
    SHOOTER_BALLISTICS_V0_REFERENCE_RPM = shooter.ballistics.v0_reference_rpm;
    SHOOTER_BALLISTICS_PREFERRED_SHOT_RPM = shooter.ballistics.preferred_shot_rpm;
    SHOOTER_BALLISTICS_MAX_SHOT_RPM = shooter.ballistics.max_shot_rpm;
    SHOOTER_BALLISTICS_V0_MARGIN_FRACTION = shooter.ballistics.v0_margin_fraction;
    SHOOTER_BALLISTICS_RPM_UPDATE_DEADBAND = shooter.ballistics.rpm_update_deadband;
    SHOOTER_BALLISTICS_FLIGHT_TIME_SEC_PER_INCH = shooter.ballistics.flight_time_sec_per_inch;
    SHOOTER_BALLISTICS_LEAD_MIN_SPEED_IPS = shooter.ballistics.lead_min_speed_ips;
    SHOOTER_SHOT_TABLE_POINTS = shooter.shot_table.points;
    AUTO_SHOT_TIME_POINTS = auto.shot_time_points;
    AUTO_BALLS_PER_SHOT_COUNT = auto.balls_per_shot_count;
    AUTO_DRINK_WAIT_MS = auto.drink_wait_ms;
    AUTO_SHOOT_WAIT_MS = auto.shoot_wait_ms;
    AUTO_LAUNCH_POWER = auto.launch_power;
    AUTO_TRANSFER_POWER = auto.transfer_power;
    AUTO_OPPOSITE_DRINK_WAIT_MS = auto.opposite_drink_wait_ms;
    AUTO_OPPOSITE_SHOOT_WAIT_MS = auto.opposite_shoot_wait_ms;
    AUTO_OPPOSITE_LAUNCH_POWER = auto.opposite_launch_power;
    AUTO_OPPOSITE_TRANSFER_POWER = auto.opposite_transfer_power;
    TELEOP_MAX_SPEED = teleop.max_speed;
    TELEOP_FIELD_CENTRIC = teleop.field_centric;
    TELEOP_FIELD_CENTRIC_OFFSET_DEG = teleop.field_centric_offset_deg;
    TELEOP_INTAKE_POWER = teleop.intake_power;
    TELEOP_TRANSFER_POWER = teleop.transfer_power;
    TELEOP_MANUAL_REV_POWER = teleop.manual_rev_power;
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
    /** Enable/disable this subsystem, controller, watchdog, or feature. */
    public boolean enabled;

    /**
     * Maximum power limit (0.0 to 1.0) commanded to the turret CRServo. Limits maximum turret
     * rotation speed. Minimum: 0.0 Maximum: 1.0
     */
    public double max_power_output;

    /**
     * Flips which CRServo direction counts as increasing turret angle. Set this (not the analog
     * encoder's 'inverted' flag) when the turret drives away from its target.
     */
    public boolean servo_direction_inverted;

    /**
     * Static friction feedforward (kS) magnitude applied when turning turret in positive direction
     * (error > 0). Minimum: 0.0 Maximum: 1.0
     */
    public double ks_positive;

    /**
     * Static friction feedforward (kS) magnitude applied when turning turret in negative direction
     * (error < 0). Minimum: 0.0 Maximum: 1.0
     */
    public double ks_negative;

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

    public static final class Travel {
      /**
       * Most negative turret angle (degrees, relative to the chassis) the turret is allowed to
       * reach. Minimum: -180.0 Maximum: 0.0
       */
      public double min_angle;

      /**
       * Most positive turret angle (degrees, relative to the chassis) the turret is allowed to
       * reach. Minimum: 0.0 Maximum: 180.0
       */
      public double max_angle;
    }

    public Travel travel;

    public static final class Stall {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Commanded turret power above which the stall watchdog starts checking for motion. Below
       * this the servo may legitimately not move, so no stall is inferred. Minimum: 0.0 Maximum:
       * 1.0
       */
      public double power_threshold;

      /**
       * Turret motion (degrees) that counts as progress and resets the stall watchdog. Set above
       * the analog encoder's noise floor. Minimum: 0.0
       */
      public double min_progress_deg;

      /**
       * Seconds of commanded power with no turret motion before the stall watchdog latches a fault
       * and cuts power. Protects the turret from being driven into a hard stop when the encoder
       * reading is wrong. Minimum: 0.0
       */
      public double timeout_sec;
    }

    public Stall stall;

    public static final class Runaway {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * How far (degrees) the turret's aim error may grow beyond its best-so-far value before the
       * runaway watchdog cuts power. Catches an inverted control loop, where positive power moves
       * the turret away from its target, before it reaches a hard stop. Minimum: 0.0
       */
      public double divergence_deg;
    }

    public Runaway runaway;
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
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
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

      /**
       * Full electrical span (volts) of the turret potentiometer, i.e. the voltage at which it
       * wraps back to zero. Minimum: 0.0
       */
      public double full_scale_voltage;

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

      /** Z value coordinate. */
      public double z;
    }

    public CameraPosition camera_position;

    public static final class CameraOrientation {
      /** Yaw rotation (degrees). */
      public double yaw;

      /** Pitch rotation (degrees). */
      public double pitch;

      /** Roll rotation (degrees). */
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
     * Time horizon (seconds) used to project future rotation based on live angular velocity
     * (|omega| * time). Seeded at 0.075s (derived from 0.45rad / assumed 6.0rad/s max omega);
     * calibrate on robot using Friction Calibration TeleOp. Minimum: 0.0
     */
    public double rotation_lookahead_time;

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
    /**
     * Enable/disable depth (X-axis) goal zone proximity protection. Safety policy parameter; leave
     * true outside of debugging.
     */
    public boolean enable_depth_protection;

    /**
     * Enable/disable side (Y-axis) goal zone proximity protection. Safety policy parameter; leave
     * true outside of debugging.
     */
    public boolean enable_side_protection;

    public static final class Repulsion {
      /**
       * Motor power (0.0 to 1.0) commanded to push the robot away when it violates a goal zone
       * boundary. Driver/safety policy parameter. Minimum: 0.0 Maximum: 1.0
       */
      public double power;

      /**
       * Safety margin factor (0.0 to 1.0) for predictive braking (e.g. 0.7 = 30% safety margin over
       * physics model prediction). Risk tolerance/policy parameter. Minimum: 0.0 Maximum: 1.0
       */
      public double decel_safety_factor;
    }

    public Repulsion repulsion;

    public static final class Depth {
      /**
       * Distance (inches) from goal zone at which proximity speed reduction starts. Driver comfort
       * / early-warning feel parameter. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from goal zone where drive power towards zone is cut to zero. Safety
       * policy parameter. Minimum: 0.0
       */
      public double hard_stop;
    }

    public Depth depth;

    public static final class Side {
      /**
       * Distance (inches) from goal zone at which proximity speed reduction starts. Driver comfort
       * / early-warning feel parameter. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from goal zone where drive power towards zone is cut to zero. Safety
       * policy parameter. Minimum: 0.0
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
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Raw joystick turn threshold (0.0 to 1.0) above which driver active steering takes control
       * and disengages heading lock. Minimum: 0.0 Maximum: 1.0
       */
      public double intent_threshold;

      /**
       * Minimum rotational drive power (0.0 to 1.0) needed to overcome friction while the robot is
       * already translating. Minimum: 0.0 Maximum: 1.0
       */
      public double ks_moving;

      /**
       * Chassis translational speed threshold (inches/sec) above which full ks_moving friction
       * feedforward is applied. Minimum: 0.0
       */
      public double moving_speed_threshold;

      /**
       * Maximum motor power (0.0 to 1.0) the loop is allowed to command. Clamped to be completely
       * imperceptible. Minimum: 0.0
       */
      public double max_power;

      /**
       * Heading error (degrees) below which heading lock applies zero correction, to avoid
       * chattering/jitter from sensor noise while holding a heading at rest. Minimum: 0.0 Maximum:
       * 15.0
       */
      public double error_deadband_deg;

      /**
       * Rotation rate (degrees/sec) below which the robot counts as settled. Heading lock waits for
       * this before capturing the heading to hold, so it does not latch mid-coast and fight the
       * robot's own momentum. Minimum: 0.0
       */
      public double settle_rate_dps;
    }

    public HeadingLock heading_lock;
  }

  public static final class Shooter {
    /**
     * When true, uses the REV firmware velocity PID on the motor (RUN_USING_ENCODER with
     * setVelocity, gains from shooter.motor_pidf). When false, uses the custom Pedro PIDF
     * controller, anti-windup integrator, and voltage compensation (RUN_WITHOUT_ENCODER, gains from
     * shooter.pidf).
     */
    public boolean use_ftc_pid;

    public double ks;

    /**
     * Bus voltage the flywheel feedforward (ks and pidf.f) was characterized at. Shooter.periodic()
     * scales its motor command by nominal_voltage divided by the measured bus voltage, so a sagging
     * pack still reaches the commanded wheel speed instead of quietly landing short and failing the
     * feed gate. Set this to the voltage shown while the Shooter Characterization OpMode is
     * running, not to a fresh-off-the-charger reading. Minimum: 1.0 Maximum: 20.0
     */
    public double nominal_voltage;

    /**
     * Largest factor the flywheel voltage compensation may scale its command by. Bounds the
     * correction so a brownout or a bad voltage reading cannot rail the flywheel; 1.4 covers a 12.0
     * V feedforward down to about 8.6 V. Minimum: 1.0 Maximum: 3.0
     */
    public double max_voltage_compensation;

    /**
     * How often the flywheel re-reads bus voltage. The hub's voltage is a separate ADC round trip
     * rather than part of the bulk read, so polling it every loop costs milliseconds per tick; a
     * pack does not sag meaningfully inside this interval. Minimum: 0.0 Maximum: 5.0
     */
    public double voltage_refresh_sec;

    /**
     * Maximum rotational speed (RPM) of the motor. Used to scale power percentage to velocity.
     * Minimum: 0.0
     */
    public double max_rpm;

    /**
     * Continuous fixed flywheel target speed (RPM). Flywheel holds this RPM constantly so
     * time-to-shoot collapses to alignment time. Minimum: 0.0 Maximum: 3000.0
     */
    public double constant_rpm;

    /**
     * Flywheel speed threshold (percentage, 0.0 to 1.0) required to trigger the transfer belt.
     * Prevents launching rings before the shooter is up to speed. Minimum: 0.0 Maximum: 1.0
     */
    public double min_transfer_threshold;

    /**
     * Flywheel velocity upper bound scale factor (e.g. 1.05) to prevent overshooting before feeding
     * rings. Applies only when arming a feed, not while one is already running, since a ring can
     * only ever slow the flywheel down. Minimum: 1.0 Maximum: 2.0
     */
    public double max_velocity_threshold;

    /**
     * Fraction of target flywheel speed a feed already in progress may sag to before the intake
     * cuts out. Must be below min_transfer_threshold: it is the lower half of a hysteresis band, so
     * each ring entering the flywheel does not drop the speed under the arming floor and stutter
     * the intake off and back on. Raise it toward min_transfer_threshold for stricter shot energy,
     * lower it if the intake still hitches while feeding. Minimum: 0.0 Maximum: 1.0
     */
    public double feed_release_threshold;

    /**
     * Front intake power (0.0 to 1.0) when feeding rings to the shooter. Minimum: 0.0 Maximum: 1.0
     */
    public double feed_intake_power;

    /**
     * Transfer motor power (0.0 to 1.0) when feeding rings to the shooter. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double feed_transfer_power;

    /**
     * Launch power ratio threshold above which the high hood position is selected for long-range
     * shots. Minimum: 0.0 Maximum: 1.0
     */
    public double long_hood_power_threshold;

    public static final class BallDetection {
      /**
       * How far flywheel speed must fall below its running reference, as a fraction, to count as a
       * ball leaving the shooter. Measured on this robot: real balls dip 12.2%-26.4%, the deepest
       * non-ball (the wheel coasting down after a shot) reaches 5.4%. Keep this between them; see
       * FlywheelDipDetector. Minimum: 0.0 Maximum: 1.0
       */
      public double dip_fraction;

      /**
       * How far flywheel speed must climb back up off a dip's own trough, as a fraction, before the
       * next ball can register. Must be strictly below dip_fraction. Ending a dip on a rebound off
       * the trough — not on a return to the pre-ball reference — is what lets balls that arrive
       * close together each get counted; see FlywheelDipDetector. Minimum: 0.0 Maximum: 1.0
       */
      public double rebound_fraction;

      /**
       * EMA weight per control loop for the ball detector's running reference speed. Small enough
       * that one ball cannot drag the reference down with it, large enough to track a flywheel
       * settling into a new cruise speed between shots. Minimum: 0.0 Maximum: 1.0
       */
      public double baseline_alpha;

      /**
       * Minimum spacing (milliseconds) between counted ball events. One ball produces a dip with a
       * ragged floor; without this, noise on the way back up would count as a second ball. Minimum:
       * 0.0
       */
      public int refractory_ms;
    }

    public BallDetection ball_detection;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    /**
     * REV firmware velocity PIDF (p, i, d, f) applied directly to the flywheel motors via
     * DcMotorEx.setPIDFCoefficients when shooter.use_ftc_pid is true. Distinct from shooter.pidf,
     * which only feeds the software PIDFController used when use_ftc_pid is false and is never sent
     * to the motor. Keep i at 0 here: a live integral on the motor firmware winds up during a
     * feed's velocity sag and overshoots the setpoint once the ball leaves, which is what stalled
     * the third shot of a magazine.
     */
    public com.qualcomm.robotcore.hardware.PIDFCoefficients motor_pidf;

    public static final class Integral {
      /**
       * Flywheel velocity error (ticks/s) within which the integral term accumulates. Outside this
       * band the integrator holds its value, so spin-up and ball-feed transients never charge it.
       * Minimum: 0.0 Maximum: 2000.0
       */
      public double band_ticks;

      /**
       * Maximum absolute motor power the flywheel integral term may contribute. Bounds windup
       * independently of the integral gain. Minimum: 0.0 Maximum: 1.0
       */
      public double max_contribution;
    }

    public Integral integral;

    public static final class Ballistics {
      /** Fitted initial ring exit velocity (in/s) at constant RPM. Minimum: 0.0 */
      public double v0;

      /**
       * Fitted lumped quadratic drag coefficient (1/in) for 2D trajectory integration. Minimum: 0.0
       */
      public double k;

      /**
       * Fitted lumped Magnus lift coefficient (1/s) for 2D backspin trajectory integration.
       * Minimum: 0.0
       */
      public double magnus_l;

      /** Physical ring release height (inches) above field carpet floor. Minimum: 0.0 */
      public double launch_height_inches;

      /**
       * Center height (inches) of target scoring goal basket above field carpet floor. Minimum: 0.0
       */
      public double goal_height_inches;

      /** Gravitational acceleration constant (in/s²). Minimum: 0.0 */
      public double g_inches_per_sec2;

      /** Minimum physical hood elevation angle (degrees). Minimum: 0.0 Maximum: 90.0 */
      public double min_hood_angle_deg;

      /** Maximum physical hood elevation angle (degrees). Minimum: 0.0 Maximum: 90.0 */
      public double max_hood_angle_deg;

      /**
       * Continuous servo position (0.0 to 1.0) corresponding to min_hood_angle_deg, the flattest
       * arc. The hood linkage is reversed, so this is numerically GREATER than max_hood_servo_pos;
       * that is expected and the angle mapping handles it. Do not 'fix' the ordering. Minimum: 0.0
       * Maximum: 1.0
       */
      public double min_hood_servo_pos;

      /**
       * Continuous servo position (0.0 to 1.0) corresponding to max_hood_angle_deg, the steepest
       * arc. Servo 0.0 is as tall as the arc gets, so this is normally the smaller number of the
       * pair. Minimum: 0.0 Maximum: 1.0
       */
      public double max_hood_servo_pos;

      /** Maximum allowed robot translation speed (in/s) for valid moving shots. Minimum: 0.0 */
      public double max_moving_speed_ips;

      /** Minimum calibrated shot distance (inches) for valid solution. Minimum: 0.0 */
      public double min_valid_distance;

      /** Maximum calibrated shot distance (inches) for valid solution. Minimum: 0.0 */
      public double max_valid_distance;

      /** Empirical calibration multiplier for virtual lead target calculation. Minimum: 0.0 */
      public double lead_bias_gain;

      /**
       * Flywheel setpoint the fitted v0 exit velocity was measured at. Exit speed and flywheel
       * setpoint convert through v0 / v0_reference_rpm, which is how the solver turns a required
       * exit speed into an RPM. Must match the RPM the ballistics calibration actually ran at
       * (constant_rpm), or every solved RPM is scaled wrong. Minimum: 1.0 Maximum: 3000.0
       */
      public double v0_reference_rpm;

      /**
       * Flywheel setpoint the hood aims around. The solver holds this speed and lets the hood alone
       * cover every distance it can, so the flywheel stays parked at one well-calibrated,
       * already-spun-up speed. It only rises above this when no hood angle can reach the goal, and
       * never falls below it. This is the main shot-strength knob: raise it if shots land weak
       * across the board. Minimum: 0.0 Maximum: 3000.0
       */
      public double preferred_shot_rpm;

      /**
       * Highest flywheel setpoint the RPM solver may command, for distances past the hood's
       * authority at preferred_shot_rpm. A target that cannot reach goal height even at this speed
       * is reported as an invalid solution instead of being silently under-shot. Minimum: 0.0
       * Maximum: 3000.0
       */
      public double max_shot_rpm;

      /**
       * Headroom above the slowest exit speed that still reaches the goal (0.12 = 12% faster),
       * applied only to long shots that had to raise RPM past preferred_shot_rpm. Keeps those
       * solutions off the boundary where the flat and lofted trajectory branches merge; larger
       * values give a steeper, more forgiving entry angle at the cost of a hotter shot. Minimum:
       * 0.0 Maximum: 1.0
       */
      public double v0_margin_fraction;

      /**
       * Minimum change in solved RPM (ticks/s) before the flywheel setpoint is re-commanded. Every
       * setpoint write discards the flywheel's accumulated anti-windup integral, so without a
       * deadband a continuously drifting target would keep the integrator reset and the flywheel
       * permanently short of speed. Minimum: 0.0
       */
      public double rpm_update_deadband;

      /**
       * Ring time of flight per inch of range, used only to lead a moving shot. A flat estimate on
       * purpose: the measured shot table records hood and RPM but not flight time, and lead only
       * shifts the aim azimuth slightly, so a rough value is adequate here where a wrong hood
       * position would not be. Minimum: 0.0 Maximum: 0.1
       */
      public double flight_time_sec_per_inch;

      /**
       * Robot speed below which moving-shot lead is skipped entirely. A robot holding position
       * still reports a small noisy velocity, and lead turns that noise into aim-azimuth jitter
       * larger than the turret's settle tolerance, so the turret hunts instead of settling. Leading
       * a nearly-stationary robot gains nothing. Minimum: 0.0 Maximum: 100.0
       */
      public double lead_min_speed_ips;
    }

    public Ballistics ballistics;

    public static final class ShotTable {
      /**
       * Measured shot table as flat distance_in, hood_servo, flywheel_rpm triples, recorded by the
       * Ballistics Calibration OpMode. Distances must strictly increase. Looked up by linear
       * interpolation between rows and clamped outside them; a distance outside the calibrated
       * range is reported as an invalid solution rather than extrapolated.
       */
      public double[] points;
    }

    public ShotTable shot_table;
  }

  public static final class Auto {
    /**
     * Measured shoot windows as flat distance_in, shoot_window_ms pairs, recorded by the Shot
     * Timing Tuner OpMode. Distances must strictly increase. Looked up by linear interpolation and
     * clamped outside the measured range, since a shot just past the last row does not suddenly get
     * faster. ShotController ends a scoring cycle when it counts auto.balls_per_shot_count balls OR
     * this window expires, whichever comes first — the window is the safety net for a jam, not the
     * primary signal. auto.shoot_wait_ms is only the fallback when this table is empty.
     */
    public double[] shot_time_points;

    /**
     * Balls loaded before each autonomous scoring cycle. ShotController counts them live via
     * shooter.ball_detection and ends the shot the moment this many are detected, without waiting
     * out the rest of auto.shot_time_points' window. Minimum: 1.0
     */
    public int balls_per_shot_count;

    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. Minimum: 0.0 */
    public int drink_wait_ms;

    /**
     * Fallback wait time (milliseconds) allowed to fire a shot, used only when
     * auto.shot_time_points is empty. Prefer the measured table: a single flat number cannot
     * express that a close-range magazine takes longer than a long-range one. Minimum: 0.0
     */
    public int shoot_wait_ms;

    public double launch_power;

    /**
     * Target power coefficient (0.0 to 1.0) for autonomous transfer motor. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double transfer_power;

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
  }

  public static final class Teleop {
    /**
     * Maximum speed scaler (0.0 to 1.0) applied to chassis manual TeleOp drive inputs. Minimum: 0.0
     * Maximum: 1.0
     */
    public double max_speed;

    /**
     * When true, driver translation sticks are interpreted in the field frame (stick direction is a
     * fixed field direction regardless of robot heading). When false, sticks are robot-relative.
     */
    public boolean field_centric;

    /**
     * Field heading (degrees) that a fully-forward driver stick should drive toward when
     * field_centric is true. 90 means stick-forward drives along field +Y, i.e. away from the
     * driver wall at y=0. Set this to whatever direction is 'away' from where the drivers
     * physically stand. Minimum: -360.0 Maximum: 360.0
     */
    public double field_centric_offset_deg;

    /** Target motor power (0.0 to 1.0) for intake rollers in TeleOp. Minimum: 0.0 Maximum: 1.0 */
    public double intake_power;

    /**
     * Target power coefficient (0.0 to 1.0) for autonomous transfer motor. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double transfer_power;

    /**
     * Target flywheel power ratio (0.0 to 1.0) for manual rev command in TeleOp. Minimum: 0.0
     * Maximum: 1.0
     */
    public double manual_rev_power;

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
    /**
     * Measured shoot windows as flat distance_in, shoot_window_ms pairs, recorded by the Shot
     * Timing Tuner OpMode. Distances must strictly increase. Looked up by linear interpolation and
     * clamped outside the measured range, since a shot just past the last row does not suddenly get
     * faster. ShotController ends a scoring cycle when it counts auto.balls_per_shot_count balls OR
     * this window expires, whichever comes first — the window is the safety net for a jam, not the
     * primary signal. auto.shoot_wait_ms is only the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Balls loaded before each autonomous scoring cycle. ShotController counts them live via
     * shooter.ball_detection and ends the shot the moment this many are detected, without waiting
     * out the rest of auto.shot_time_points' window.
     */
    public int ballsPerShotCount;

    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. */
    public int drinkWaitMs;

    /**
     * Fallback wait time (milliseconds) allowed to fire a shot, used only when
     * auto.shot_time_points is empty. Prefer the measured table: a single flat number cannot
     * express that a close-range magazine takes longer than a long-range one.
     */
    public int shootWaitMs;

    public double launchPower;

    /** Target power coefficient (0.0 to 1.0) for autonomous transfer motor. */
    public double transferPower;

    /** Wait delay (milliseconds) at the drink gate for opposite alliance autonomous. */
    public int oppositeDrinkWaitMs;

    /** Wait time (milliseconds) allowed to fire a shot for opposite alliance autonomous. */
    public int oppositeShootWaitMs;

    /** Target power coefficient (0.0 to 1.0) for opposite alliance autonomous launcher. */
    public double oppositeLaunchPower;

    /** Target power coefficient (0.0 to 1.0) for opposite alliance autonomous transfer. */
    public double oppositeTransferPower;
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
    /**
     * Measured shoot windows as flat distance_in, shoot_window_ms pairs, recorded by the Shot
     * Timing Tuner OpMode. Distances must strictly increase. Looked up by linear interpolation and
     * clamped outside the measured range, since a shot just past the last row does not suddenly get
     * faster. ShotController ends a scoring cycle when it counts auto.balls_per_shot_count balls OR
     * this window expires, whichever comes first — the window is the safety net for a jam, not the
     * primary signal. auto.shoot_wait_ms is only the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Balls loaded before each autonomous scoring cycle. ShotController counts them live via
     * shooter.ball_detection and ends the shot the moment this many are detected, without waiting
     * out the rest of auto.shot_time_points' window.
     */
    public int ballsPerShotCount;

    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. */
    public int drinkWaitMs;

    /**
     * Fallback wait time (milliseconds) allowed to fire a shot, used only when
     * auto.shot_time_points is empty. Prefer the measured table: a single flat number cannot
     * express that a close-range magazine takes longer than a long-range one.
     */
    public int shootWaitMs;

    public double launchPower;

    /** Target power coefficient (0.0 to 1.0) for autonomous transfer motor. */
    public double transferPower;
  }

  public static MatchProfile loadMatchProfile(Alliance alliance) {
    return MatchProfile.loadMatchProfile(alliance);
  }
}
