// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeCommand extends Command {

  private IntakeSubsystem m_intakeSubsystem;

  public IntakeCommand() {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (SubsystemContainer.armSubsystem.getTargetAngle() == Constants.ArmConstants.ARM_LOWER_LIMIT
        && SubsystemContainer.armSubsystem.getInPosition()) {
      m_intakeSubsystem.intakeNote();
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
