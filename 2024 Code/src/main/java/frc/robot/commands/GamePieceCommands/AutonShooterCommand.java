// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;

public class AutonShooterCommand extends GamePieceCommand {

  private double m_rpm;
  private CalculateAngle m_angleCalculate;
  private SwerveSubsystem m_swerve;

  public AutonShooterCommand(double p_rpm) {
    m_swerve = SubsystemContainer.swerveSubsystem;
    m_rpm = p_rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(m_rpm);
    m_armSubsystem.setArmReferenceAngle(m_angleCalculate.InterpolateAngle(m_swerve.GetSpeakerToRobot()));
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
    return m_intakeSubsystem.hasNoNote();
  }
}
