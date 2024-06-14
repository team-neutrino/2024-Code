// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateMovingShot;
import frc.robot.util.SubsystemContainer;

public class TranslatedAutoAlignCommand extends Command {
  private CalculateMovingShot m_calculateMovingShot;

  /**
   * A command for left/right shoot while move calculations.
   */
  public TranslatedAutoAlignCommand(CalculateMovingShot p_calculateMovingShot) {
    m_calculateMovingShot = p_calculateMovingShot;

    addRequirements(SubsystemContainer.limelightSubsystem);
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
      double movementOffset = m_calculateMovingShot.calculateAdjustedPos().getTheta();

      // System.out.println("adjustment angle: " + movementOffset);
      System.out.println(
          "\"true\" if autoaligned: " + SubsystemContainer.swerveSubsystem.AutoAligned());

      SubsystemContainer.swerveSubsystem
          .setRobotYaw(
<<<<<<< Updated upstream
              SwerveSubsystem.calculateLimelightOffsetAngle() + movementOffset); // TODO: CHANGE SIGN AS NEEDED
=======
              SwerveSubsystem.calculateLimelightOffsetAngle() + movementOffset);
>>>>>>> Stashed changes

    } else {
      System.out.println("Limelight can't see a tag...");
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
