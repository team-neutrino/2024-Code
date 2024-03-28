// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShuttleCloseCommand extends Command {
  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  public ShuttleCloseCommand() {
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_shooterSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(ShooterSpeeds.SHUTTLE_CLOSE_SPEED);
    if (m_intakeSubsystem.isBeamBrokenIntake()
        || m_intakeSubsystem.isBeamBrokenIndex() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndexShoot();
    } else {
      m_intakeSubsystem.smartIntake();
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
