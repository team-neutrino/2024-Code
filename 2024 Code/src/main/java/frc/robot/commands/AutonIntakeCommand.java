// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutonIntakeCommand extends Command {

  private IntakeSubsystem m_intakeSubsystem;
  private Timer m_timer;
  private ArmSubsystem m_armSubsystem;
  private double m_angle;
  private ShooterSubsystem m_shooterSubsystem;
  private double m_rpm;

  public AutonIntakeCommand(double p_angle, double p_rpm) {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_angle = p_angle;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_rpm = p_rpm;
    addRequirements(m_intakeSubsystem, m_armSubsystem, m_shooterSubsystem);
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
    m_intakeSubsystem.smartIntake();
    m_shooterSubsystem.setTargetRPM(m_rpm);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    return ((SubsystemContainer.intakeSubsystem.hasNote()) || m_timer.get() > 3);
  }
}
