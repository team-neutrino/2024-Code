// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class ArmManualCommand extends Command {
  private ArmSubsystem m_armSubsystem;
  private double m_shiftAngle;
  private XboxController m_controller;

  public ArmManualCommand(CommandXboxController p_controller) {
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_controller = p_controller.getHID();
    // Either save the subsystem or don't, don't mix it
    addRequirements(SubsystemContainer.armSubsystem);
  }

  @Override
  public void initialize() {
    // Here too
    // Rename shift angle to manual target angle. Its not a shift of anything
    // Why save this at init at all? Why not grab the current angle during the execute portion?
    m_shiftAngle = SubsystemContainer.armSubsystem.getArmAngleDegrees();
  }

  @Override
  public void execute() {
    // What if someone changes the arms angle inbetween here and initialize?
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
