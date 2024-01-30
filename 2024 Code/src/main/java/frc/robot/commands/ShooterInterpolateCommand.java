// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.CalculateRPM;

public class ShooterInterpolateCommand extends Command {
  CalculateRPM m_RPMCalculate;

  public ShooterInterpolateCommand(CalculateRPM p_RPMCalculate) {
    m_RPMCalculate = p_RPMCalculate;
    addRequirements(SubsystemContainer.ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.ShooterSubsystem.setTargetRPM(m_RPMCalculate.InterpolateRPM());
    System.out.println(SubsystemContainer.ShooterSubsystem.getTargetRPM());
    System.out.println(SubsystemContainer.armSubsystem.getCurrentAngle());
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
