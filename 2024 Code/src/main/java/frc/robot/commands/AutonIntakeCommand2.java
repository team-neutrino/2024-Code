// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutonIntakeCommand2 extends Command {

  int i;
  private IntakeSubsystem m_intakeSubsystem;

  public AutonIntakeCommand2() {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    i = 0;
  }

  @Override
  public void execute() {
    m_intakeSubsystem.stopIntake();

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.isBeamBrokenIntake()) {
      i++;
      if (i > 10) {
        return true;
      }
    }
    return false;
  }
}
