// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonMagicSpeakerCommand extends GamePieceCommand {
  CalculateAngle m_calculateAngle;
  private SwerveSubsystem m_swerve;
  int i = 0;

  public AutonMagicSpeakerCommand(CalculateAngle p_calculateAngle) {
    m_calculateAngle = p_calculateAngle;
    m_swerve = SubsystemContainer.swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle(m_swerve.GetSpeakerToRobot()));
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
    if (!m_intakeSubsystem.isBeamBrokenIntake()) {
      i++;
      if (i > 10) {
        return true;
      }
    }
    return false;
  }
}