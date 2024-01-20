// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.util.SubsystemContainer;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_controller = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_controller);
  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_IntakeDefaultCommand = new IntakeDefaultCommand();
  ShooterDefaultCommand m_ShooterDefaultCommand = new ShooterDefaultCommand();
  ClimbDefaultCommand m_climbDefaultCommand = new ClimbDefaultCommand();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // set default commands
    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);
    SubsystemContainer.swerveSubsystem.setDefaultCommand(m_swerveDefaultCommand);
    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_IntakeDefaultCommand);
    SubsystemContainer.climbSubsystem.setDefaultCommand(m_climbDefaultCommand);
    SubsystemContainer.ShooterSubsystem.setDefaultCommand(m_ShooterDefaultCommand);

    // LED buttons
    m_controller.a().whileTrue(new LEDCommand());

    // Intake buttons
    m_controller.y().whileTrue(new IntakeReverseCommand());

    // Climb buttons
    m_controller.x().whileTrue(new ClimbExtendCommand());
    m_controller.leftTrigger().whileTrue(new ClimbRetractCommand());
    m_controller.b().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));
    m_controller.leftBumper().onTrue(new PathPlannerAuto("New Auto"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
