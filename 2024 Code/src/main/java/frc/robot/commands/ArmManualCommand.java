// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.SubsystemContainer;

public class ArmManualCommand extends Command {
  private double m_shiftAngle;
  private XboxController m_controller;
  double cycle = 0;

  public ArmManualCommand(CommandXboxController p_controller) {
    m_controller = p_controller.getHID();
    addRequirements(SubsystemContainer.armSubsystem);
  }

  @Override
  public void initialize() {
    m_shiftAngle = SubsystemContainer.armSubsystem.getArmAngleDegrees();
  }

  @Override
  public void execute() {
    cycle++;
    if (Math.abs(m_controller.getLeftX()) > ArmConstants.ARM_ADJUST_DEADZONE) {
      m_shiftAngle -= m_controller.getLeftX();
    }

    if (cycle % 8 == 0) {
      System.out.println("shift angle " + m_shiftAngle);
    }

    SubsystemContainer.armSubsystem.setArmReferenceAngle(m_shiftAngle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
