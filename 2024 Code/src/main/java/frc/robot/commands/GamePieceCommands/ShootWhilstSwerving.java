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
import frc.robot.util.PolarCoord;
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
public class ShootWhilstSwerving extends GamePieceCommand {

  private SwerveSubsystem m_swerveSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private CalculateAngle m_calculateAngle;
  private CommandXboxController m_driverController;
  private CommandXboxController m_buttonsController;

  /** Creates a new ShootWhileSwerving. */
  public ShootWhilstSwerving(CommandXboxController p_driverController, CommandXboxController p_buttonsController) {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_calculateAngle = SubsystemContainer.m_angleCalculate;
    m_driverController = p_driverController;
    m_buttonsController = p_buttonsController;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.SwerveWithoutDeadzone(0, m_driverController.getLeftX() * -1, 0);

    PolarCoord adjustedSpeakerToRobot = calculateAdjustedPos();

    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle(adjustedSpeakerToRobot));
    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHOOTING_SPEED);
    m_intakeSubsystem.runIndexFeed();
  }

  private PolarCoord calculateAdjustedPos() {
    double r = m_swerveSubsystem.GetSpeakerToRobot().getRadius();
    double theta = m_swerveSubsystem.GetSpeakerToRobot().getTheta();
    double robotSpeed = m_swerveSubsystem.getDriveMotorSpeed();

    double deltaX = (r / ShooterConstants.NOTE_SPEED) * robotSpeed;

    return new PolarCoord(r, calculateAdjustedTheta(r, theta, deltaX));
  }

  private double calculateAdjustedTheta(double r, double theta, double deltaX) {
    double adjustedRadius = calculateAdjustedRadius(r, theta, deltaX);

    return Math.asin((deltaX * Math.sin(theta)) / adjustedRadius);
  }

  private double calculateAdjustedRadius(double r, double theta, double deltaX) {
    return Math.sqrt((Math.pow(r, 2) + Math.pow(deltaX, 2)) - (2 * r * deltaX * Math.cos(theta)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_buttonsController.getHID().getLeftBumper() && m_armSubsystem.getInPosition()
        && m_shooterSubsystem.approveShoot() && m_intakeSubsystem.isNoteReady();
  }
}
