// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.util.CalculateRPM;

public class ShooterInterpolateCommand extends GamePieceCommand {
  private CalculateRPM m_RPMCalculate;

  public ShooterInterpolateCommand(CalculateRPM p_RPMCalculate) {
    m_RPMCalculate = p_RPMCalculate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(m_RPMCalculate.InterpolateRPM());
    if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndexShoot();
    } else {
      m_intakeSubsystem.stopIndex();
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
