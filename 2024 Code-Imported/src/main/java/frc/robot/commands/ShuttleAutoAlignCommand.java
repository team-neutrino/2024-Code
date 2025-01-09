// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.util.SubsystemContainer;

public class ShuttleAutoAlignCommand extends Command {
  /** Creates a new ShuttleAutoAlignCommand. */

  private XboxController m_xboxController;

  public ShuttleAutoAlignCommand(CommandXboxController p_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (p_controller != null) {
      m_xboxController = p_controller.getHID();
    }
    addRequirements(SubsystemContainer.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.swerveSubsystem.AlignToCornerUsingOdometry();

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
