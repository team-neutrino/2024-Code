// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.Fidelity;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.PolarCoord;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.SwerveRequestStash;

/* 
 * x is front-back and y is side side, y is negated. So: 
 * (-.5, .5) = back left
 * (-.5, -.5) = back right
 * (.5, .5) = front left
 * (.5, -.5) = front right
*/
public class EscapeManeuvers extends Command {
  private CommandXboxController controller;

  /** Creates a new KrakenSwerveEscapeManeuver. */
  public EscapeManeuvers(CommandXboxController p_controller) {
    controller = p_controller;
    addRequirements(SubsystemContainer.swerveSubsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerX = controller.getLeftX();
    double controllerY = -controller.getLeftY();
    double fieldRelativeMoveAngle = Math.atan2(controllerY, controllerX);
    double yaw = Math.toRadians(SubsystemContainer.swerveSubsystem2.getYaw());

    Rotation2d robotRelativeMoveAngle = new Rotation2d(fieldRelativeMoveAngle - yaw - (Math.PI / 2));

    SubsystemContainer.swerveSubsystem2
        .setControl(
            SwerveRequestStash.drive.withCenterOfRotation(new Translation2d(.5, robotRelativeMoveAngle))
                .withVelocityX(controller.getLeftY() * SwerveConstants.MaxSpeed)
                .withVelocityY(controller.getLeftX() * SwerveConstants.MaxSpeed)
                .withRotationalRate(controller.getRightX() *
                    SwerveConstants.MaxAngularRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
