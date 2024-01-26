package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  public static class OperatorConstants {
    public static final int XBOX_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
  }

  /**
   * Climber motors controller id's (the first parameter in the construction line)
   * are 40s.
   */
  public static class ClimbConstants {
    public static final double CLIMB_EXTEND_MOTOR_SPEED = .2; // PLACEHOLDER VALUE
    public static final double CLIMB_RETRACT_MOTOR_SPEED = -.2; // PLACEHOLDER VALUE
    public static final double CLIMB_LIMIT_UP = 50; // PLACEHOLDER VALUE
  }

  public static class DimensionConstants {
    public static double WHEEL_DIAMETER = 0.1016;
    public static double WHEEL_CIRCUMFERENCE = 0.1016 * Math.PI;
  }

  public static class SwerveConstants {
    public static double GEAR_RATIO = 6.55;
    public static double ANGLE_P = 0.01;
    public static double SPEED_P = 0.01;

    public static Translation2d FRONT_RIGHT_COORD = new Translation2d(0.2155261469, -0.2155261469);
    public static Translation2d FRONT_LEFT_COORD = new Translation2d(0.2155261469, 0.2155261469);
    public static Translation2d BACK_RIGHT_COORD = new Translation2d(-0.2155261469, -0.2155261469);
    public static Translation2d BACK_LEFT_COORD = new Translation2d(-0.2155261469, 0.2155261469);
    public static double DRIVEBASE_RADIUS = 0.304799999992;

    public static double MAX_CHASSIS_LINEAR_SPEED = 1;
    public static double MAX_CHASSIS_ROTATIONAL_SPEED = 1.5 * Math.PI;
    public static double MAX_MODULE_ROTATION_SPEED;
    public static double MAX_MODULE_LINEAR_SPEED = 7;

    public static double FLA_OFFSET = 43;
    public static double FRA_OFFSET = 336.6;
    public static double BLA_OFFSET = 82.7;
    public static double BRA_OFFSET = 359.2;

    public static double ks = 0.15;
    public static double kv = 2.6;

    public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(1, 1, 1.5 * Math.PI, 4 * Math.PI);

    public static Translation2d AMP_TARGET_POSE_RED = new Translation2d(1.9304, 1);
    public static Translation2d AMP_TARGET_POSE_BLUE = new Translation2d(12.7254, 1);

    public static Translation2d TWO_NOTE_TARGET_POSE_RED = new Translation2d();
    public static Translation2d TWO_NOTE_TARGET_POSE_BLUE = new Translation2d();
    public static Translation2d CLOSE_NOTE_TARGET_POSE_RED = new Translation2d();
    public static Translation2d CLOSE_NOTE_TARGET_POSE_BLUE = new Translation2d();
    public static Translation2d CLOSE_THEN_MID_TARGET_POSE_RED = new Translation2d();
    public static Translation2d CLOSE_THEN_MID_TARGET_POSE_BLUE = new Translation2d();
    public static Translation2d MID_TARGET_POSE_RED = new Translation2d();
    public static Translation2d MID_TARGET_POSE_BLUE = new Translation2d();

    public static Pose2d BLUE_TARGET_POSE1 = new Pose2d(2, 4, new Rotation2d());
    public static Pose2d BLUE_TARGET_POSE2 = new Pose2d(0.1, 4.8, new Rotation2d());
    public static Pose2d BLUE_TARGET_POSE3 = new Pose2d(2.1, 2.1, new Rotation2d());
    public static Pose2d BLUE_TARGET_POSE4 = new Pose2d();

    public static Pose2d RED_TARGET_POSE1;
    public static Pose2d RED_TARGET_POSE2;
    public static Pose2d RED_TARGET_POSE3;
    public static Pose2d RED_TARGET_POSE4;

    public static HashMap<Command, Translation2d> m_blueCoordMap = new HashMap<>();
    public static HashMap<Command, Translation2d> m_redCoordMap = new HashMap<>();
  }

  public final class DigitalConstants {
    public static int INTAKE_MOTOR_BEAMBREAK = 0;
    public static int SHOOTER_BEAMBREAK = 1;
    public static int ARM_ENCODER = 2;

  }

  public final class PWMConstants {
    public static int LED = 0;
  }

  public final class LEDConstants {
    public static int LEDBufferLen = 60;
  }

  public final class MotorIDs {
    // A means Angular and S means Speed.

    public static int FLA = 4;
    public static int FRA = 2;
    public static int BLA = 8;
    public static int BRA = 6;

    public static int FLS = 3;
    public static int FRS = 1;
    public static int BLS = 7;
    public static int BRS = 5;

    public static int Arm = 10;

    public static int INTAKE_MOTOR = 20;
    public static int INDEX_MOTOR = 21;

    public static int SHOOTER_MOTOR = 30;

    public static final int CLIMB_MOTOR1 = 40;
    public static final int CLIMB_MOTOR2 = 41;
  }

  public class ArmConstants {
    public static double Arm_kp = 0.8;
    public static double Arm_ki = 0.01;
    public static double Arm_kd = 0.07;
    public static double INTAKE_LIMIT = 95;
    public static double AMP_LIMIT = 15;
    public static double INTAKE_POSE = 90;
    public static double AMP_POSE = 20;
    public static double ARM_ADJUST_DEADZONE = 0.2;
    public static double CLIMB_POSITION = 30; // PLACEHOLDER VALUE
  }

  public final class IntakeConstants {
    public static double INTAKE_MOTOR_SPEED = 0.2;
    public static double INDEX_MOTOR_SPEED = 0.1;
  }
}
