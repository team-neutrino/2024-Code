// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.util.SubsystemContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_controller = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_controller);
  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_IntakeDefaultCommand = new IntakeDefaultCommand();

  // JoystickButton m_buttonX = new JoystickButton(m_controller,
  // XboxController.Button.kX.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {

    m_controller.a().whileTrue(new LEDCommand());

    m_controller.b().whileTrue(new IntakeCommand());

    m_controller.y().whileTrue(new IntakeReverseCommand());

    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);

    SubsystemContainer.swerveSubsystem.setDefaultCommand(m_swerveDefaultCommand);

    m_controller.x().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));
    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_IntakeDefaultCommand);

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    m_controller.a().onTrue(new PathPlannerAuto("New Auto"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
