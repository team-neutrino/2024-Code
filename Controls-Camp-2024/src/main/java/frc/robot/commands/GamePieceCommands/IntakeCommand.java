// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

public class IntakeCommand extends GamePieceCommand {
  public IntakeCommand() {
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_shooterSubsystem.defaultShooter();
    m_armSubsystem.defaultArm();
    m_intakeSubsystem.smartIntake();

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
