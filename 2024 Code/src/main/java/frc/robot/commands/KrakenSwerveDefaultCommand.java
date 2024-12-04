// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.SwerveRequestStash;

public class KrakenSwerveDefaultCommand extends Command {
  CommandXboxController m_controller;

  /** Creates a new KrakenSwerveDefaultCommand. */
  public KrakenSwerveDefaultCommand(CommandXboxController controller) {
    m_controller = controller;

    addRequirements(SubsystemContainer.swerveSubsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.swerveSubsystem2.setControl(SwerveRequestStash.drive
        .withVelocityX(m_controller.getLeftY() * SwerveConstants.MaxSpeed)
        .withVelocityY(m_controller.getLeftX() * SwerveConstants.MaxSpeed)
        .withRotationalRate(-m_controller.getRightX() *
            SwerveConstants.MaxAngularRate));
    // SubsystemContainer.swerveSubsystem2.setControl(SwerveRequestStash.drive1
    // .withVelocityX(m_controller.getLeftY() * SwerveConstants.MaxSpeed)
    // .withVelocityY(m_controller.getLeftX() * SwerveConstants.MaxSpeed)
    // .withTargetDirection(new Rotation2d(0)));
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
