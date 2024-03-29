// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class MagicAmpChargeCommand extends GamePieceCommand {
  private CommandXboxController m_controller;

  /** Creates a new MagicAmpChargeCommand. */
  public MagicAmpChargeCommand(CommandXboxController p_xboxcontroller) {
    m_controller = p_xboxcontroller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(Constants.ArmConstants.AMP_POSE);
    m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.AMP_SPEED);
    m_intakeSubsystem.runIndexFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot() && m_controller.getHID().getLeftBumper();
  }
}
