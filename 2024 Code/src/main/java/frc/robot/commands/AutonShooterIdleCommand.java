// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutonShooterIdleCommand extends Command {

  private ShooterSubsystem m_shooterSubsystem;
  private double m_rpm;

  public AutonShooterIdleCommand(double p_rpm) {
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_rpm = p_rpm;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(m_rpm);
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
