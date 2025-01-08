// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants.LEDConstants.States;

public class ArmClimbCommandUp extends GamePieceCommand {
  /** Creates a new ArmClimbCommandUp. */
  public ArmClimbCommandUp() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(90);
    m_intakeSubsystem.defaultIntake();
    m_shooterSubsystem.setTargetRPM(0);
    m_armSubsystem.setCommandState(States.CLIMBING);

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
