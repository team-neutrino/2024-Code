// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDefaultCommand extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private XboxController m_xboxController;

  public SwerveDefaultCommand(CommandXboxController p_controller) {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_xboxController = p_controller.getHID();
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.Swerve(m_xboxController.getLeftY() * -1,
        m_xboxController.getLeftX() * -1,
        m_xboxController.getRightX() * -1);
    m_swerveSubsystem.setCommandState(States.DEFAULT);

    // D-pad addition: pressing any of the 4 main buttons on the D-pad
    // serve as hotkeys for rotation to forward, backward, left, and right
    // relative to field orientation.
    float pov = m_xboxController.getPOV();
    if (pov >= 0) {
      switch (Math.round(pov)) {
        case (0):
          m_swerveSubsystem.setRobotYaw(0);
          break;
        case (90):
          m_swerveSubsystem.setRobotYaw(-90);
          break;
        case (180):
          m_swerveSubsystem.setRobotYaw(180);
          break;
        case (270):
          m_swerveSubsystem.setRobotYaw(270);
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.setCommandState(States.PATHFINDING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}