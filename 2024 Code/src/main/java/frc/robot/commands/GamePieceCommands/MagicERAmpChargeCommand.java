// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.util.SubsystemContainer;

public class MagicERAmpChargeCommand extends GamePieceCommand {

  /** Creates a new MagicAmpChargeCommand. */
  public MagicERAmpChargeCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.commandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(SubsystemContainer.swerveSubsystem.getAmpDy());
    if (SubsystemContainer.swerveSubsystem.getAmpDy() < .5) {
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
        && m_shooterSubsystem.aboveRPM(Constants.ShooterSpeeds.LOW_SPEED_THRESHOLD);
  }
}
