// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;

public class MagicERAmpChargeCommand extends GamePieceCommand {

  /** Creates a new MagicAmpChargeCommand. */
  public MagicERAmpChargeCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SubsystemContainer.swerveSubsystem.getAmpDy() < SwerveConstants.AMP_SHOOTING_ZONE) {
      m_armSubsystem.setArmReferenceAngle(Constants.ArmConstants.AMP_POSE);
      m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.AMP_SPEED);
    } else {
      m_armSubsystem.defaultArm();
      m_shooterSubsystem.defaultShooter();
    }
    m_intakeSubsystem.stopIntake();
    m_intakeSubsystem.runIndexFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.aboveAngle(Constants.ArmConstants.AMP_THRESHOLD)
        && m_shooterSubsystem.aboveRPM(Constants.ShooterSpeeds.AMP_SPEED_THRESHOLD);
  }
}
