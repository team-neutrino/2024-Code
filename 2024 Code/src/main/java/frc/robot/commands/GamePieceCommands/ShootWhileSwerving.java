// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;

/**
 * This (1st) iteration will only support movement to the left and right
 * completely parallel to the speaker.
 * 
 * This command is to be run in parallel with the AutoAlignCommand.
 * 
 * Future incremental design plan: 4 directions, 8 directions, omnidirectional
 * linear, left and right curved path (U path [while spinning??]),
 * omnidirectional quadratic.
 */
public class ShootWhileSwerving extends GamePieceCommand {

  private SwerveSubsystem m_swerveSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private CalculateAngle m_calculateAngle;
  private CommandXboxController m_controller;

  /** Creates a new ShootWhileSwerving. */
  public ShootWhileSwerving(CommandXboxController p_controller) {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_calculateAngle = SubsystemContainer.m_angleCalculate;
    m_controller = p_controller;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.SwerveWithoutDeadzone(0, m_controller.getLeftX() * -1, 0);
    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle(m_swerveSubsystem.GetSpeakerToRobot()));
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
    return false;
  }
}
