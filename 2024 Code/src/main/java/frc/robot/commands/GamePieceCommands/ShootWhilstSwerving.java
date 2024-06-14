// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateAngle;
import frc.robot.util.CalculateMovingShot;
import frc.robot.util.PolarCoord;
import frc.robot.util.SubsystemContainer;

/**
 * This (1st) iteration will only support movement backwards and forwards
 * completely perpendicular to the speaker.
 * 
 * This command is to be run in parallel with the AutoAlignCommand.
 * 
 * Future incremental design plan: not perpendiular forward/backward,
 * left/right fixed radius (semicircle), left/right non-fixed radius,
 * cardinal directions, 8 directions, omnidirectional linear.
 */
public class ShootWhilstSwerving extends GamePieceCommand {

  private ShooterSubsystem m_shooterSubsystem;
  private CalculateAngle m_calculateAngle;
  private CommandXboxController m_controller;
  private PolarCoord adjustedSpeakerToRobot;
  private CalculateMovingShot m_calculateMovingShot;

  /** Creates a new ShootWhileSwerving. */
  public ShootWhilstSwerving(CommandXboxController p_controller, CalculateMovingShot p_calculateMovingShot) {
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_calculateAngle = SubsystemContainer.m_angleCalculate;
    m_controller = p_controller;
    m_calculateMovingShot = p_calculateMovingShot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
    // future fixed-radius calculations below vvv
    // adjustedSpeakerToRobot = m_calculateMovingShot.calculateAdjustedPos();

    m_armSubsystem
        .setArmReferenceAngle(m_calculateAngle.InterpolateAngle(m_calculateMovingShot.adjustBackwardMovement()));
    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHOOTING_SPEED);
=======
    // adjustedSpeakerToRobot = m_calculateMovingShot.calculateAdjustedPos();

    m_armSubsystem.setArmReferenceAngle(
        m_calculateAngle.InterpolateAngle(SubsystemContainer.swerveSubsystem.GetSpeakerToRobot()));

    double shootingSpeed = Math.max(5000, 4000 + (SubsystemContainer.swerveSubsystem.getDriveMotorSpeed() / 0.0508));
    m_shooterSubsystem.setTargetRPM(shootingSpeed);

    m_intakeSubsystem.runIndexFeed();
>>>>>>> Stashed changes
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.getHID().getLeftBumper() && m_armSubsystem.getInPosition()
        && m_shooterSubsystem.approveShoot();
  }
}
