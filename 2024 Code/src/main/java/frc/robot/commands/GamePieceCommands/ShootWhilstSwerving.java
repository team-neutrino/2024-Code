// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateAngle;
import frc.robot.util.CalculateMovingShot;
import frc.robot.util.SubsystemContainer;

/**
 * Change in thought process once again, but to reiterate: modifying the arm
 * angle for movement would affect note apagee too much with a fixed speed and
 * NEOs aren't powerful enough to modify hte note speed. So, this attempt will
 * try to scrap the interpolation table and instead use an equation relating
 * radial distance form the speaker to the robot to arm angle. See
 * "AntiInterpolationEquation" in CalculateMovingShot for more details.
 * 
 * Must be run while auto-aligning to work.
 */
public class ShootWhilstSwerving extends GamePieceCommand {

  private ShooterSubsystem m_shooterSubsystem;
  private CalculateAngle m_calculateAngle;
  private CommandXboxController m_controller;
  private SwerveSubsystem m_swerve = SubsystemContainer.swerveSubsystem;
  private LimelightSubsystem m_limelight = SubsystemContainer.limelightSubsystem;

  /** Creates a new ShootWhileSwerving. */
  public ShootWhilstSwerving(CommandXboxController p_controller) {
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_calculateAngle = SubsystemContainer.m_angleCalculate;
    m_controller = p_controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_swerve.withinMovingShootingDistance()) {
      System.out.println("OUTSIDE OF SHOOTING RANGE");
      return;
    }

    m_shooterSubsystem.setTargetRPM(ShooterSpeeds.SHOOTING_SPEED);
    m_intakeSubsystem.runIndexFeed();

    double robotSpeed = SubsystemContainer.swerveSubsystem.getRadialSpeed();
    // double angle =
    // m_calculateAngle.InterpolateAngle(SubsystemContainer.swerveSubsystem.GetSpeakerToRobot());
    double angle = CalculateMovingShot
        .adjustArmForMovement(robotSpeed, SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getRadius());

    System.out.println("Auto-aligned = " + SubsystemContainer.swerveSubsystem.AutoAligned());

    m_armSubsystem.setArmReferenceAngle(angle);
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
        m_swerve.withinMovingShootingDistance() &&
        m_swerve.robotVelocityWithinTolerance() &&
        // only previous conditions below
        m_armSubsystem.getInPosition() &&
        m_shooterSubsystem.aboveRPM(ShooterSpeeds.THRESHOLD_SHOOTING_SPEED) &&
        m_intakeSubsystem.isNoteReady();
  }
}
