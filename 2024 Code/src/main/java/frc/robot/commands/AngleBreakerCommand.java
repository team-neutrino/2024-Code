// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.util.SubsystemContainer;

public class AngleBreakerCommand extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private ArmSubsystem m_armSubsystem;
  private CommandXboxController controller;

  /** Creates a new AngleBreakerCommand. */
  public AngleBreakerCommand(CommandXboxController p_controller) {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    controller = p_controller;

    addRequirements(m_swerveSubsystem, m_shooterSubsystem, m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getRightX() >= .5) {

    } else if (controller.getRightX() <= -.5) {

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
