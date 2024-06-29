// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.GamePieceCommands.ArmManualCommand;
import frc.robot.commands.GamePieceCommands.IntakeCommand;
import frc.robot.commands.GamePieceCommands.IntakeReverseCommand;
import frc.robot.commands.GamePieceCommands.MagicAmpChargeCommand;
import frc.robot.commands.GamePieceCommands.MagicShootCommand;
import frc.robot.commands.GamePieceCommands.ShootManualCommand;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_buttonsController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand(m_buttonsController);
  IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();

  public RobotContainer() {
    configureBindings();

    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());
  }

  private void configureBindings() {
    // set default commands
    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);
    SubsystemContainer.swerveSubsystem.setDefaultCommand(new SwerveDefaultCommand(m_driverController));
    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    SubsystemContainer.armSubsystem.setDefaultCommand(new ArmDefaultCommand());
    SubsystemContainer.shooterSubsystem.setDefaultCommand(new ShooterDefaultCommand());

    // Intake buttons
    m_driverController.leftBumper().whileTrue(new IntakeReverseCommand());
    m_driverController.leftTrigger().whileTrue(new IntakeCommand());

    // swerve buttons
    m_driverController.back().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));

    // m_driverController.b().onTrue(new InstantCommand(() -> {
    // SubsystemContainer.swerveSubsystem.ResetModules();
    // SubsystemContainer.armSubsystem.initializeMotorControllers();
    // }));

    // shooter buttons
    m_buttonsController.a()
        .whileTrue(new SequentialCommandGroup(new MagicAmpChargeCommand(m_buttonsController), new MagicShootCommand()));

    m_buttonsController.x().whileTrue(new SequentialCommandGroup(
        new ShootManualCommand(Constants.ArmConstants.SUBWOOFER_ANGLE, Constants.ShooterSpeeds.SHOOTING_SPEED,
            Constants.ShooterSpeeds.LOW_SPEED_THRESHOLD, m_buttonsController),
        new MagicShootCommand()));

    m_buttonsController.povDown()
        .onTrue(new InstantCommand(() -> SubsystemContainer.armSubsystem.initializeMotorControllers()));

    // arm buttons
    m_buttonsController.leftStick().toggleOnTrue(new ArmManualCommand(m_buttonsController));
  }

  public void teleopperiodic() {
  }
}
