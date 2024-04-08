// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateMovingShot;
import frc.robot.util.SubsystemContainer;

public class TranslatedAutoAlignCommand extends Command {
  private CalculateMovingShot m_calculateMovingShot;

  /** Creates a new TranslatedAutoAlignCommand. */
  public TranslatedAutoAlignCommand(CalculateMovingShot p_calculateMovingShot) {
    m_calculateMovingShot = p_calculateMovingShot;

    addRequirements(SubsystemContainer.limelightSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SubsystemContainer.limelightSubsystem.getTv()) {
      SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();

      double currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
      double offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
      double movementOffset = m_calculateMovingShot.calculateAdjustedPos().getTheta();

      System.out.println("adjustment angle: " + movementOffset);

      double[] pose = SubsystemContainer.limelightSubsystem.getBotPose();
      if (!SubsystemContainer.alliance.isRedAlliance()) {
        if (pose[5] > 0) {
          pose[5] -= 180;
        } else {
          pose[5] += 180;
        }
      }

      SubsystemContainer.swerveSubsystem
          .setRobotYaw(
              SwerveSubsystem.calculateLimelightOffsetAngle(currentYaw, offsetYaw + movementOffset, pose[5]));

    } else {
      // SUPER auto align!!
      SubsystemContainer.swerveSubsystem.AlignToSpeakerUsingOdometry();
    }

    SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);
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
