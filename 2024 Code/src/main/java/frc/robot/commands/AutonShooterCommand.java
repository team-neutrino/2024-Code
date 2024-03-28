// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;

public class AutonShooterCommand extends Command {

  // We are crazy inconsistent with our access modifiers
  private ShooterSubsystem m_shooterSubsystem;
  private double m_rpm;
  private ArmSubsystem m_armSubsystem;
  private SwerveSubsystem m_swerve;
  private CalculateAngle m_angleCalculate;
  private IntakeSubsystem m_intakeSubsystem;

  public AutonShooterCommand(double p_rpm, CalculateAngle p_angleCalculate) {
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_rpm = p_rpm;
    m_angleCalculate = p_angleCalculate;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_swerve = SubsystemContainer.swerveSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_shooterSubsystem, m_armSubsystem, m_intakeSubsystem);
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
    // Shouldn't we check the other beam break here?
    return SubsystemContainer.intakeSubsystem.noNote();
  }
}
