// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

public class AutonArmCommand extends GamePieceCommand {
  double m_angle;

  public AutonArmCommand(double p_angle) {
    m_angle = p_angle;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_angle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
