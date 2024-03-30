// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.Timer;

public class IndexJitterCommand extends GamePieceCommand {
  private Timer m_timer;

  public IndexJitterCommand() {
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.indexJitter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > 2 && m_intakeSubsystem.isBeamBrokenIndex());
  }
}
