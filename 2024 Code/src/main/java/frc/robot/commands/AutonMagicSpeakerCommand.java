// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.CalculateAngle;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonMagicSpeakerCommand extends Command {
  CalculateAngle m_calculateAngle;
  ArmSubsystem m_armSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  int i = 0;

  public AutonMagicSpeakerCommand(CalculateAngle p_calculateAngle) {
    m_calculateAngle = p_calculateAngle;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_armSubsystem, m_shooterSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_calculateAngle.InterpolateAngle());
    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle());
    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHOOTING_SPEED);
    if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndexShoot();
    } else {
      m_intakeSubsystem.stopIndex();
    }
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
      if (i > 10) {
        return true;
      }
    }
    return false;
  }
}