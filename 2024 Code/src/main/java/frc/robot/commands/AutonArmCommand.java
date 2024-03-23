// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutonArmCommand extends Command {

  private ArmSubsystem m_armSubsystem;
  double m_angle;

  public AutonArmCommand(double p_angle) {
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_angle = p_angle;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_angle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
