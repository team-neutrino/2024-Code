// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.CalculateAngle;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MagicSpeakerChargeCommand extends Command {
  private CalculateAngle m_calculateAngle;
  private ArmSubsystem m_armSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private SwerveSubsystem m_swerve;
  private CommandXboxController m_controller;

  public MagicSpeakerChargeCommand(CalculateAngle p_calculateAngle, CommandXboxController p_controller) {
    m_calculateAngle = p_calculateAngle;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    m_swerve = SubsystemContainer.swerveSubsystem;
    m_controller = p_controller;
    addRequirements(m_armSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_intakeSubsystem);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // for testing polynomial surface
    // m_armSubsystem.setArmReferenceAngle(m_calculateAngle.quarticFitCalculateAngle(m_swerve.GetSpeakerToRobot()));
    // MAke calculate angle a static
    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle(m_swerve.GetSpeakerToRobot()));
    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHOOTING_SPEED);
    m_intakeSubsystem.runIndexFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Rename getInposition to isInPosition
    return m_controller.getHID().getLeftBumper() && m_armSubsystem.getInPosition()
        && m_shooterSubsystem.approveShoot() && m_intakeSubsystem.isCentered();
  }
}
