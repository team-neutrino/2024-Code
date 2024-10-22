// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AntiInterpolationCalculation;
import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;

public class AutonShooterCommand extends GamePieceCommand {
  private double m_rpm;
  private SwerveSubsystem m_swerve;
  private CalculateAngle m_angleCalculate;

  public AutonShooterCommand(double p_rpm, CalculateAngle p_angleCalculate) {
    m_rpm = p_rpm;
    m_angleCalculate = p_angleCalculate;
    m_swerve = SubsystemContainer.swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.commandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem
        .setArmReferenceAngle(AntiInterpolationCalculation.getArmAngle(m_swerve.GetSpeakerToRobot().getRadius()) - 1.5);

    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHOOTING_SPEED);
    m_intakeSubsystem.runIndexFeed();
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
