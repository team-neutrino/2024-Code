// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class ArmAngleCommand extends Command {
  private ArmSubsystem m_armSubsystem;
  private double m_angle;

  public ArmAngleCommand(double p_angle) {
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_angle = p_angle;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_armSubsystem.getArmAngleDegrees() <= (m_angle + 5)
        && m_armSubsystem.getArmAngleDegrees() >= (m_angle - 5));

  }
}
