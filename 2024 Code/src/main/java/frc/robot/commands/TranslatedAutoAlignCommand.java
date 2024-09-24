// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShootWhilstSwerveConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class TranslatedAutoAlignCommand extends AutoAlignCommand {

  /**
   * A command for left/right shoot while move calculations.
   */
  public TranslatedAutoAlignCommand(CommandXboxController p_controller) {
    super(p_controller);
  }

  // superclass "initialize" is used.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!SubsystemContainer.limelightSubsystem.getTv()) {
      System.out.println("no target");
      return;
    }

    SubsystemContainer.swerveSubsystem
        .setRobotYaw(SwerveSubsystem.calculateLimelightOffsetAngle() + ShootWhilstSwerveConstants.AUTO_ALIGN_FLICK);

    // basic swerve driving from here to bottom
    SubsystemContainer.swerveSubsystem.SwerveWithDeadzone(m_xboxController.getLeftY() * -1,
        m_xboxController.getLeftX() * -1,
        m_xboxController.getRightX() * -1);

    SubsystemContainer.swerveSubsystem.POV(m_xboxController.getPOV());

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
