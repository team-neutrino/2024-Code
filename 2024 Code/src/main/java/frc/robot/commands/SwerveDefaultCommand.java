// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SubsystemContainer;

public class SwerveDefaultCommand extends Command {
  XboxController m_xboxController;
  double cycle;

  public SwerveDefaultCommand(CommandXboxController p_controller) {
    m_xboxController = p_controller.getHID();
    addRequirements(SubsystemContainer.swerveSubsystem);
    cycle = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cycle++;
    if (cycle % 8 == 0) {
      // System.out.println("controller y " + m_xboxController.getLeftY());
      // System.out.println("controller x " + m_xboxController.getRightX());
    }

    SubsystemContainer.swerveSubsystem.Swerve(m_xboxController.getLeftY() * -1, m_xboxController.getLeftX() * -1,
        m_xboxController.getRightX() * -1);
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