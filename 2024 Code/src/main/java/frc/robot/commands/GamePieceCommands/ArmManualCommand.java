// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;;

public class ArmManualCommand extends GamePieceCommand {
  private double m_targetAngle;
  private XboxController m_controller;

  public ArmManualCommand(CommandXboxController p_controller) {
    m_controller = p_controller.getHID();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_targetAngle = m_armSubsystem.getArmAngleDegrees();

    if (Math.abs(m_controller.getLeftX()) > ArmConstants.ARM_ADJUST_DEADZONE) {
      m_targetAngle -= m_controller.getLeftX();
    }

    m_targetAngle = m_armSubsystem.limitArmAngle(m_targetAngle);

    m_armSubsystem.setArmReferenceAngle(m_targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
