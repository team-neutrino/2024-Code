// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LEDDefaultCommand;
//import frc.robot.commands.ShooterdefaultCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  XboxController m_driverController = new XboxController(OperatorConstants.XBOX_CONTROLLER);

  XboxController m_controller = new XboxController(OperatorConstants.XBOX_CONTROLLER);

  SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_controller);
  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();

  private final JoystickButton m_buttonA = new JoystickButton(m_controller, XboxController.Button.kA.value);

  private XboxController m_OperatorController = new XboxController(Constants.ControllerConstants.XBOX_CONTROLLER_ID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_buttonA.whileTrue(new LEDCommand());

    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);

    SubsystemContainer.swerveSubsystem.setDefaultCommand(m_swerveDefaultCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
