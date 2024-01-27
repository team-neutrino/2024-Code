// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ClimbCommand extends Command {

  CommandXboxController m_controller;

  ClimbSubsystem m_climbSubsystem = SubsystemContainer.climbSubsystem;

  ArmSubsystem m_armSubsystem = SubsystemContainer.armSubsystem;

  /**
   * Creates an instance of ClimbCommand, requires the xbox controller
   * due to annoyances with the arm default command. Ask Nathan if you
   * want to know more.
   * 
   * @param p_controller Button monkey xbox controller.
   */
  public ClimbCommand(CommandXboxController p_controller) {
    m_controller = p_controller;
    addRequirements(m_armSubsystem, m_climbSubsystem);
  }

  /**
   * Since this is a toggled command, it is better that
   * the arm goes to the climb position as soon as the
   * command is scheduled.
   */
  @Override
  public void initialize() {
    m_armSubsystem.armPID(Constants.ArmConstants.CLIMB_POSITION);
  }

  /**
   * if the arm is not in the climb position, this command
   * still allows the driver to extend the arms, just not
   * retract them. Currently using the same controller
   * deadzone as armAdjust.
   */
  @Override
  public void execute() {

    double controllerPos = m_controller.getRightY();

    if (controllerPos > Constants.ArmConstants.ARM_ADJUST_DEADZONE && m_armSubsystem.getInPosisition()) {
      m_climbSubsystem.retractClimberArms();

    } else if (controllerPos < -Constants.ArmConstants.ARM_ADJUST_DEADZONE) {
      m_climbSubsystem.extendClimberArms();

    } else {
      m_climbSubsystem.stopClimberArms();

    }

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
