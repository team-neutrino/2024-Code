// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.MagicAmpChargeCommand;
import frc.robot.commands.MagicShootCommand;
import frc.robot.commands.MagicSpeakerChargeCommand;
import frc.robot.commands.ShootManualCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShuttleCloseCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.AutoAlignSequentialCommand;
import frc.robot.commands.AutonArmAngleCommand;
import frc.robot.commands.AutonArmCommand;
import frc.robot.commands.AutonArmInterpolateAngle;
import frc.robot.commands.AutonFeederCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonMagicSpeakerCommand;
import frc.robot.commands.AutonShooterCommand;
import frc.robot.commands.AutonShooterIdleCommand;
import frc.robot.commands.AutonSingleShotCommand;
import frc.robot.commands.ArmAngleCommand;
import frc.robot.commands.ArmClimbCommandDown;
import frc.robot.commands.ArmClimbCommandUp;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignForeverCommand;
import frc.robot.commands.IndexJitterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.util.SubsystemContainer;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_buttonsController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
  LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();

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
    SubsystemContainer.armSubsystem.setDefaultCommand(new ArmAngleCommand(ArmConstants.INTAKE_POSE));
    SubsystemContainer.shooterSubsystem.setDefaultCommand(new ShooterDefaultCommand());
    SubsystemContainer.limelightSubsystem.setDefaultCommand(m_LimelightDefaultCommand);

    // set named commands
    NamedCommands.registerCommand("MagicSpeakerCommand",
        new AutonMagicSpeakerCommand(SubsystemContainer.m_angleCalculate));
    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand());
    NamedCommands.registerCommand("IntakeDefaultCommand", m_intakeDefaultCommand);
    NamedCommands.registerCommand("AutoAlignCommand", new AutoAlignSequentialCommand());
    NamedCommands.registerCommand("ArmDown", new AutonArmAngleCommand(ArmConstants.INTAKE_POSE));
    NamedCommands.registerCommand("SingleSubwooferShot",
        new AutonSingleShotCommand(ArmConstants.SUBWOOFER_ANGLE, Constants.ShooterSpeeds.SHOOTING_SPEED));
    NamedCommands.registerCommand("AutonIntakeCommand", new AutonIntakeCommand());
    NamedCommands.registerCommand("AutonFeederCommmand", new AutonFeederCommand());
    NamedCommands.registerCommand("AutonShooterIdle",
        new AutonShooterIdleCommand(Constants.ShooterSpeeds.INITIAL_SHOOTER_SPEED));
    NamedCommands.registerCommand("AutonShoot",
        new AutonShooterCommand(Constants.ShooterSpeeds.SHOOTING_SPEED));
    NamedCommands.registerCommand("AutonArmDown", new AutonArmCommand(Constants.ArmConstants.INTAKE_POSE));
    NamedCommands.registerCommand("AutonArmInterpolate",
        new AutonArmInterpolateAngle(SubsystemContainer.m_angleCalculate));
    NamedCommands.registerCommand("AutoAlignForever", new AutoAlignForeverCommand());

    // Intake buttons
    m_driverController.leftBumper().whileTrue(new IntakeReverseCommand());
    m_driverController.rightTrigger().whileTrue(new ShuttleCloseCommand());
    m_driverController.leftTrigger().whileTrue(new SequentialCommandGroup(
        new IntakeCommand(), new IndexJitterCommand()));

    // swerve buttons
    m_driverController.back().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));

    m_driverController.b().onTrue(new InstantCommand(() -> {
      for (int i = 0; i < 4; i++) {
        SubsystemContainer.swerveSubsystem.swerveModules[i].resetEverything();
      }
      SubsystemContainer.armSubsystem.initializeMotorControllers();
    }));

    // shooter buttons
    m_buttonsController.a()
        .whileTrue(new SequentialCommandGroup(new MagicAmpChargeCommand(m_buttonsController), new MagicShootCommand()));

    m_driverController.a().whileTrue(new InstantCommand(() -> SubsystemContainer.m_angleCalculate.dumpData()));

    m_driverController.start()
        .whileTrue(new InstantCommand(() -> SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose()));

    // separate button binding to left bumper contained within the magic speaker
    // charge command
    m_buttonsController.y().whileTrue(new SequentialCommandGroup(
        new MagicSpeakerChargeCommand(SubsystemContainer.m_angleCalculate, m_buttonsController),
        new MagicShootCommand()));

    m_buttonsController.x().whileTrue(new SequentialCommandGroup(
        new ShootManualCommand(Constants.ArmConstants.SUBWOOFER_ANGLE, Constants.ShooterSpeeds.SHOOTING_SPEED,
            m_buttonsController),
        new MagicShootCommand()));

    m_buttonsController.b()
        .whileTrue(new SequentialCommandGroup(new ShootManualCommand(Constants.ArmConstants.SHUTTLE_ANGLE,
            Constants.ShooterSpeeds.SHUTTLE_SPEED, m_buttonsController), new MagicShootCommand()));

    m_buttonsController.povDown()
        .onTrue(new InstantCommand(() -> SubsystemContainer.armSubsystem.initializeMotorControllers()));

    m_driverController.rightBumper().whileTrue(new AutoAlignCommand());

    // arm buttons
    m_buttonsController.leftStick().toggleOnTrue(new ArmManualCommand(m_buttonsController));
    m_buttonsController.back().toggleOnTrue(new ArmClimbCommandDown());
    m_buttonsController.start().toggleOnTrue(new ArmClimbCommandUp());
  }

  public Command getAutonomousCommand() {
    Command auto;
    try {
      auto = new PathPlannerAuto("1 Note and Mid");
    } catch (Exception e) {
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }

  public void teleopperiodic() {
  }
}
