// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.SwerveRequestStash;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
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
    if (controller.getRightY() > .2) {
      SubsystemContainer.swerveSubsystem2
          .setControl(SwerveRequestStash.drive.withCenterOfRotation(new Translation2d(5, -5)).withVelocityY(-.1)
              .withRotationalRate(Math.PI));
    } else if (controller.getRightY() < -.2) {
      SubsystemContainer.swerveSubsystem2
          .setControl(SwerveRequestStash.drive.withCenterOfRotation(new Translation2d(5, -5)).withVelocityY(.1)
              .withRotationalRate(-Math.PI));
    }
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
