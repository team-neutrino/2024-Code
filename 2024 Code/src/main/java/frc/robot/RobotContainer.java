// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignSequentialCommand;
import frc.robot.commands.ArmAngleCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.util.SubsystemContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_controller = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
  ShooterDefaultCommand m_ShooterDefaultCommand = new ShooterDefaultCommand();
  ClimbDefaultCommand m_climbDefaultCommand = new ClimbDefaultCommand();
  LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // set default commands
    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);
    SubsystemContainer.swerveSubsystem.setDefaultCommand(new SwerveDefaultCommand(m_controller));
    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    SubsystemContainer.climbSubsystem.setDefaultCommand(m_climbDefaultCommand);
    SubsystemContainer.armSubsystem.setDefaultCommand(new ArmAngleCommand(50));
    SubsystemContainer.ShooterSubsystem.setDefaultCommand(m_ShooterDefaultCommand);
    SubsystemContainer.limelightSubsystem.setDefaultCommand(m_LimelightDefaultCommand);

    // LED buttons
    m_controller.a().whileTrue(new LEDCommand());

    // Intake buttons
    m_controller.y().whileTrue(new IntakeReverseCommand());

    // Climb buttons
    m_controller.leftTrigger().whileTrue(new ClimbRetractCommand());

    m_controller.start().whileTrue(new ArmAngleCommand(Constants.ArmConstants.AMP_POSE));

    // swerve buttons
    m_controller.b().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));

    m_controller.start().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> SubsystemContainer.swerveSubsystem.updatePathfindCommand()),
            SubsystemContainer.swerveSubsystem.m_pathfind, new AutoAlignSequentialCommand()));

    m_controller.leftBumper().onTrue(new PathPlannerAuto("New Auto"));

    m_controller.rightBumper().onTrue(AutoBuilder.pathfindToPose(
        SubsystemContainer.swerveSubsystem.getPathfindingTargetPose(), SwerveConstants.PATH_CONSTRAINTS));
    //m_controller.rightBumper().whileTrue(new AutoAlignCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
