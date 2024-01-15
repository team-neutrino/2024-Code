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
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  CommandXboxController m_controller = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_controller);
  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_IntakeDefaultCommand = new IntakeDefaultCommand();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {

    m_controller.a().whileTrue(new LEDCommand());

    m_controller.b().whileTrue(new IntakeCommand());

    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);

    SubsystemContainer.swerveSubsystem.setDefaultCommand(m_swerveDefaultCommand);

    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_IntakeDefaultCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
