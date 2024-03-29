// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.Timer;

public class AutonIntakeCommand extends GamePieceCommand {
  private Timer m_timer;
  private double m_angle;
  private double m_rpm;

  public AutonIntakeCommand(double p_angle, double p_rpm) {
    m_angle = p_angle;
    m_rpm = p_rpm;
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_angle);
    m_intakeSubsystem.intakeNote();
    m_shooterSubsystem.setTargetRPM(m_rpm);
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
