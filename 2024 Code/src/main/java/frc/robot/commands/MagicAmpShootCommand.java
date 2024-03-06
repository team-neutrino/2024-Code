// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class MagicAmpShootCommand extends Command {

  private int i = 0;
  private IntakeSubsystem m_intakeSubsystem = SubsystemContainer.intakeSubsystem;

  /** Creates a new MagicAmpCommand. */
  public MagicAmpShootCommand() {

    addRequirements(SubsystemContainer.armSubsystem, SubsystemContainer.shooterSubsystem,
        m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.runIndexShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_intakeSubsystem.isBeamBroken()) {
      i++;
    } else {
      i = 0;
    }
    if (i >= 10) {
      return true;
    }

    return false;
  }
}
