// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.awt.event.KeyEvent;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.RobotInputListener;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.SwerveRequestStash;

public class KrakenSwerveDefaultCommand extends Command {
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  CommandXboxController m_controller;

  /** Creates a new KrakenSwerveDefaultCommand. */
  public KrakenSwerveDefaultCommand(CommandXboxController controller) {
    m_controller = controller;

    addRequirements(SubsystemContainer.swerveSubsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SubsystemContainer.inputListener.updateCursor();

    SubsystemContainer.swerveSubsystem2.setControl(SwerveRequestStash.drive
        .withVelocityX(SubsystemContainer.inputListener.isInputActive(KeyEvent.VK_A) ? SwerveConstants.MaxSpeed
            : SubsystemContainer.inputListener.isInputActive(KeyEvent.VK_D) ? -SwerveConstants.MaxSpeed : 0)
        .withVelocityY(SubsystemContainer.inputListener.isInputActive(KeyEvent.VK_W) ? -SwerveConstants.MaxSpeed
            : SubsystemContainer.inputListener.isInputActive(KeyEvent.VK_S) ? SwerveConstants.MaxSpeed : 0)
        .withTargetDirection(getTargetAngle()));
  }

  /**
   * Only accounts for lateral mouse position. Returns a value that can be used in
   * a {@link SwerveRequest.FieldCentricFacingAngle}, which means that the angle
   * will be positive to the left on a unit circle shifted 90 to the left (so that
   * 0 is facing forward). Due to the rotation, the X-axis is vertical and Y-axis
   * is horizontal, i.e.: a target angle of 90 degrees from a starting point of 0
   * degrees would turn the robot to the left 90 degrees until it rested on the
   * Y-axis.
   * <p>
   * The length of the "screen" is 1546 pixels, see {@link RobotInputListener} for
   * more details. Currently this method uses a sensitivity where 1/4 the distance
   * of the screen (or half the distance from the center to the edge) is 180
   * degrees.
   * 
   * @return A {@link Rotation2d} that can be plugged into a a
   *         {@link SwerveRequest.FieldCentricFacingAngle}.
   */
  private Rotation2d getTargetAngle() {
    double mousePos = SubsystemContainer.inputListener.getMousePosition().getX() / 1546;
    return new Rotation2d((.5 - mousePos) * Math.PI);
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
