// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.Timer;;;

public class AutonIntakeCommand extends GamePieceCommand {

  private Timer m_timer;

  public AutonIntakeCommand() {
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_intakeSubsystem.intakeNote();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    return ((m_intakeSubsystem.hasNote()) || m_timer.get() > 3);
  }
}
