// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class LockSwerveYCommand extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private CommandXboxController m_controller;

  /** Creates a new LockSwerveYCommand. */
  public LockSwerveYCommand(CommandXboxController p_controller) {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_controller = p_controller;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.SwerveWithoutDeadzone(0, m_controller.getLeftX() * -1, 0);
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
