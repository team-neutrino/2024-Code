// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.SubsystemContainer;

public class MagicAmpCommand extends Command {

  /** Creates a new MagicAmpCommand. */
  public MagicAmpCommand() {

    addRequirements(SubsystemContainer.armSubsystem, SubsystemContainer.ShooterSubsystem,
        SubsystemContainer.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.armSubsystem.setArmReferenceAngle(Constants.ArmConstants.AMP_POSE);
    SubsystemContainer.ShooterSubsystem.setTargetRPM(500);
    if (SubsystemContainer.armSubsystem.getInPosition() && SubsystemContainer.ShooterSubsystem.approveShoot()) {
      SubsystemContainer.intakeSubsystem.runIndex();
    } else {
      SubsystemContainer.intakeSubsystem.stopIndex();
    }
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
