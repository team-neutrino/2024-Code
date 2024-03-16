package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int XBOX_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
  }

  public static class DimensionConstants {
    public static double WHEEL_DIAMETER = 0.1016;
    public static double WHEEL_CIRCUMFERENCE = 0.1016 * Math.PI;
    public static double SPEAKER_TO_MOUNT_HEIGHT = 0.6604;
  }

  public static class SwerveConstants {
    public static double GEAR_RATIO = 6.55;
    public static double ANGLE_P = 0.01;
    public static double SPEED_P = 0.01;

    public static Translation2d FRONT_RIGHT_COORD = new Translation2d(0.339725, -0.3175);
    public static Translation2d FRONT_LEFT_COORD = new Translation2d(0.339725, 0.3175);
    public static Translation2d BACK_RIGHT_COORD = new Translation2d(-0.339725, -0.3175);
    public static Translation2d BACK_LEFT_COORD = new Translation2d(-0.339725, 0.3175);
    public static double DRIVEBASE_RADIUS = 0.465074;

    public static double MAX_CHASSIS_LINEAR_SPEED = 3;
    public static double MAX_CHASSIS_LINEAR_SPEED_FAST = 3;
    public static double MAX_CHASSIS_ROTATIONAL_SPEED = 1.5 * Math.PI;
    public static double MAX_MODULE_ROTATION_SPEED;
    public static double MAX_MODULE_LINEAR_SPEED = 7;

    public static double FLA_OFFSET = 285;
    public static double FRA_OFFSET = 273;
    public static double BLA_OFFSET = 1;
    public static double BRA_OFFSET = 311;

    public static double ks = 0.15;
    public static double kv = 2.6;

    public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(1, 1, 1.5 * Math.PI, 4 * Math.PI);

    public static Translation2d CENTER_OF_FIELD_M = new Translation2d(8.29564, 4.105656);

    public static Translation2d SPEAKER_BLUE_SIDE = new Translation2d(0, 5.35);
    public static Translation2d SPEAKER_RED_SIDE = new Translation2d(16.5, 5.35);

    public static Translation2d AMP_TARGET_POSE_RED = new Translation2d(12.7254, 7.75);
    public static Translation2d AMP_TARGET_POSE_BLUE = new Translation2d(1.84, 7.75);

    public static Pose2d BLUE_TARGET_POSE1 = new Pose2d(2, 4, new Rotation2d());
    public static Pose2d BLUE_TARGET_POSE2 = new Pose2d(0.1, 4.8, new Rotation2d());
    public static Pose2d BLUE_TARGET_POSE3 = new Pose2d(2.1, 2.1, new Rotation2d());
    public static Pose2d BLUE_TARGET_POSE4 = new Pose2d();

    public static Pose2d RED_TARGET_POSE1;
    public static Pose2d RED_TARGET_POSE2;
    public static Pose2d RED_TARGET_POSE3;
    public static Pose2d RED_TARGET_POSE4;

    public static int ANGLE_MOTOR_CURRENT_LIMIT = 25;
    public static int SPEED_MOTOR_CURRENT_LIMIT = 40;
  }

  public final class DigitalConstants {
    public static int INTAKE_MOTOR_BEAMBREAK = 0;
    public static int INDEX_MOTOR_BEAMBREAK = 1;
    public static int ARM_ENCODER = 2;

  }

  public final class PWMConstants {
    public static int LED = 0;
  }

  public final class LEDConstants {
    public static int LEDBufferLen = 60;

    public enum States {
      DEFAULT,
      PATHFINDING,
      AUTOALIGN
    }
  }

  public final class MotorIDs {
    // A means Angular and S means Speed.

    public static int FRA = 3;
    public static int FLA = 5;
    public static int BRA = 7;
    public static int BLA = 9;

    public static int FRS = 2;
    public static int FLS = 4;
    public static int BRS = 6;
    public static int BLS = 8;

    public static int Arm = 10;

    public static int INTAKE_MOTOR = 20;
    public static int INTAKE_MOTOR_TWO = 21;
    public static int INDEX_MOTOR = 22;
    public static int INDEX_MOTOR2 = 23;

    public static int SHOOTER_MOTOR1 = 30;
    public static int SHOOTER_MOTOR2 = 31;

    public static final int CLIMB_MOTOR1 = 40;
    public static final int CLIMB_MOTOR2 = 41;
  }

  public final class ArmConstants {
    public static double Arm_kp = 0.022;
    public static double Arm_ki = 0.0;
    public static double Arm_kd = 0.0;

    public static double FF_kg = 0.007;
    public static double ARM_MASS_KG = 8.3733; // LBS 18.46
    public static double ARM_ABS_ENCODER_ZERO_OFFSET = 279;
    public static double ARM_RADIUS = 0.6555;
    public static double ARM_CM = 0.37084; // INCHES 14.6
    public static int ARM_CURRENT_LIMIT = 60;

    // TODO: FeedForward Constants are PLACEHOLDERS
    public static double FF_ks = 0.0;
    public static double FF_kv = 0.0;
    public static double FF_ka = 0.0;

    public static double ARM_UPPER_LIMIT = 105;
    public static double ARM_LOWER_LIMIT = -27;

    public static double INTAKE_LIMIT = 95;
    public static double AMP_LIMIT = 15;
    public static double INTAKE_POSE = -27;
    public static double AMP_POSE = 90;
    public static double ARM_ADJUST_DEADZONE = 0.2;
    public static double CLIMB_POSITION = 30; // PLACEHOLDER VALUE
    public static final double FERRY_ANGLE = 0; // PLACEHOLDER VALUE
    public static final double SUBWOOFER_ANGLE = -10; // PLACEHOLDER VALUE
  }

  public final class IntakeConstants {
    public static double INTAKE_MOTOR_VOLTAGE = 1;
    public static double INDEX_MOTOR_VOLTAGE_INTAKE = 0.2;
    public static double INDEX_MOTOR_VOLTAGE_POSITION = 0.05;
    public static double INDEX_MOTOR_VOLTAGE_SHOOT = 1;
    public static double INDEX_JITTER_MOTOR_VOLTAGE = 0.125;
    public static int INTAKE_CURRENT_LIMIT = 35;
    public static int INDEX_CURRENT_LIMIT = 20;
  }

  public final class ShooterSpeeds {
    public static final double SHOOTING_SPEED = 4000;
    public static final double FERRY_SPEED = 3200;
    public static final double INITIAL_SHOOTER_SPEED = 700;
  }

  public final class ShooterConstants {
    public static int SHOOTER_CURRENT_LIMIT = 40;
  }
}
