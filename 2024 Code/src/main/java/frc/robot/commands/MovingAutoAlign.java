// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.ShootWhilstSwervingMath;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.SwerveRequestStash;

public class MovingAutoAlign extends AutoAlignCommand {

  /** Creates a new MovingAutoAlign. */
  public MovingAutoAlign(CommandXboxController p_controller) {
    super(p_controller);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.swerveSubsystem2.setControl(SwerveRequestStash.drive
        .withVelocityX(m_xboxController.getLeftY() * SwerveConstants.MaxSpeed)
        .withVelocityY(m_xboxController.getLeftX() * SwerveConstants.MaxSpeed)
        .withRotationalRate(-movementCompensatedOmega()));
  }

  /**
   * Modifies the autoalign angle to account for movement tangential to the
   * speaker.
   * <p>
   * Uses a ratio of 2 m/s per radian/s of rotation.
   * 
   * @return The speed the robot should turn, accounting for movement.
   */
  private double movementCompensatedOmega() {
    double omega = offsetToOmega(SubsystemContainer.limelightSubsystem.getOffsetAngleFromTag());
    omega += ShootWhilstSwervingMath.getSpeakerRelativeTangentialVelocity() * .5;
    return omega;
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
