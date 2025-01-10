// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;

public class WrappedSwerve extends CommandSwerveDrivetrain {

  /** Creates a new WrappedSwerve. */
  public WrappedSwerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
  }

  /**
   * Gets the current yaw of the robot in degrees.
   * 
   * @return The current yaw.
   */
  public double getYaw() {
    return getPigeon2().getYaw().getValueAsDouble();
  }

  public Pose2d getCurrentPose() {
    return getState().Pose;
  }

  public void resetPigeon() {
    getPigeon2().reset();
  }

  @Override
  public void periodic() {
  }
}
