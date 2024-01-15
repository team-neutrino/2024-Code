// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int XBOX_CONTROLLER = 0;
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

    public static double FLA_OFFSET = 0;
    public static double FRA_OFFSET = 0;
    public static double BLA_OFFSET = 0;
    public static double BRA_OFFSET = 0;

    public static int FLA = 8;
    public static int FRA = 6;
    public static int BLA = 2;
    public static int BRA = 4;

    public static int FLS = 7;
    public static int FRS = 5;
    public static int BLS = 1;
    public static int BRS = 3;

    public static double ks = 0.15;
    public static double kv = 2.6;
  }

  public final class MotorConstants {
    public static final int INTAKE_MOTOR = 20;
  }

  public final class DIgitalConstats {
    public static final int INTAKE_BEAMBREAK = 0;
  }
}