package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int XBOX_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
  }

  public final class ControllerConstants {
    public static int XBOX_CONTROLLER_ID = 2;
  }

  public final class DriverConstants {
    public static double MAX_SPEED = 0.5;
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

    public static double MAX_CHASSIS_LINEAR_SPEED = 1;
    public static double MAX_CHASSIS_ROTATIONAL_SPEED = 1.5 * Math.PI;
    public static double MAX_MODULE_ROTATION_SPEED;

    public static double FLA_OFFSET = 43;
    public static double FRA_OFFSET = 336.6;
    public static double BLA_OFFSET = 82.7;
    public static double BRA_OFFSET = 359.2;

    public static double ks = 0.15;
    public static double kv = 2.6;
  }

  public final class DigitalConstants {
    public static int LED = 0;
    public static int SHOOTER_BEAMBREAK = 2;
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
    public static int shooter = 30;
    public static int INTAKE_MOTOR = 20;
  }

  public final class IntakeConstants {
    public static double INTAKE_MOTOR_SPEED = 0.1;
  }
}
