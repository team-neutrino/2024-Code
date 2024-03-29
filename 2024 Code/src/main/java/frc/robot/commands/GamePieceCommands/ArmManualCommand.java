// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;

public class ArmManualCommand extends GamePieceCommand {
  private double m_shiftAngle;
  private XboxController m_controller;

  public ArmManualCommand(CommandXboxController p_controller) {
    m_controller = p_controller.getHID();
  }

  @Override
  public void initialize() {
    m_shiftAngle = m_armSubsystem.getArmAngleDegrees();
  }

  @Override
  public void execute() {
    if (Math.abs(m_controller.getLeftX()) > ArmConstants.ARM_ADJUST_DEADZONE) {
      m_shiftAngle -= m_controller.getLeftX();
    }

    m_shiftAngle = m_armSubsystem.limitArmAngle(m_shiftAngle);

    m_armSubsystem.setArmReferenceAngle(m_shiftAngle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
