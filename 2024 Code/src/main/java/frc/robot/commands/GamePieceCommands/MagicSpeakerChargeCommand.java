// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MagicSpeakerChargeCommand extends GamePieceCommand {
  private CalculateAngle m_calculateAngle;
  private SwerveSubsystem m_swerve;
  private CommandXboxController m_controller;
  private LimelightSubsystem m_limelight;

  public MagicSpeakerChargeCommand(CalculateAngle p_calculateAngle, CommandXboxController p_controller) {
    m_calculateAngle = p_calculateAngle;
    m_swerve = SubsystemContainer.swerveSubsystem;
    m_limelight = SubsystemContainer.limelightSubsystem;
    m_controller = p_controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.commandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // for testing polynomial surface
    // m_armSubsystem.setArmReferenceAngle(m_calculateAngle.quarticFitCalculateAngle(m_swerve.GetSpeakerToRobot()));

    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle(m_swerve.GetSpeakerToRobot()));
    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHOOTING_SPEED);
    m_intakeSubsystem.runIndexFeed();
    System.out.println("autoaligned:" + m_swerve.AutoAligned());
    System.out.println("facingSpeakerID:" + m_limelight.facingSpeakerID());
    System.out.println("withinShootingDistance:" + m_swerve.withinShootingDistance());
    System.out.println("robotVelocityWithinTolerance:" + m_swerve.robotVelocityWithinTolerance());
    System.out.println("arm in position:" + m_armSubsystem.getInPosition());
    System.out.println("shooter above RPM:" + m_shooterSubsystem.aboveRPM(ShooterSpeeds.THRESHOLD_SHOOTING_SPEED));
    System.out.println("note ready:" + m_intakeSubsystem.isNoteReady());
    System.out.println();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.getHID().getLeftBumper() &&
        m_swerve.AutoAligned() &&
        m_limelight.facingSpeakerID() &&
        m_swerve.withinShootingDistance() &&
        m_swerve.robotVelocityWithinTolerance() &&
        // only previous conditions below
        m_armSubsystem.getInPosition() &&
        m_shooterSubsystem.aboveRPM(ShooterSpeeds.THRESHOLD_SHOOTING_SPEED) &&
        m_intakeSubsystem.isNoteReady();
  }
}
