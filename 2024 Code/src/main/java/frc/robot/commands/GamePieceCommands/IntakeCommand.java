// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants;

public class IntakeCommand extends GamePieceCommand {

  public IntakeCommand() {
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (m_armSubsystem.getTargetAngle() == Constants.ArmConstants.ARM_LOWER_LIMIT
        && m_armSubsystem.getInPosition()) {
      m_intakeSubsystem.intakeNote();
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
