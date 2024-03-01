// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.MagicAmpCommand;
import frc.robot.commands.MagicSpeakerCommand;
import frc.robot.commands.ShootManualCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.AutoAlignSequentialCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ArmAngleCommand;
import frc.robot.commands.ArmInterpolateCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.util.CalculateAngle;
import frc.robot.util.SubsystemContainer;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_controller = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
  ClimbDefaultCommand m_climbDefaultCommand = new ClimbDefaultCommand();
  LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();
  CalculateAngle m_angleCalculate = new CalculateAngle();
  MagicSpeakerCommand m_magicSpeakerCommand = new MagicSpeakerCommand(m_angleCalculate);
  IntakeCommand m_intakeCommand = new IntakeCommand();
  AutoAlignSequentialCommand m_autoAlignSequential = new AutoAlignSequentialCommand();
  ArmAngleCommand m_armDefaultCommand = new ArmAngleCommand(-25);

  public RobotContainer() {
    configureBindings();

    DataLogManager.start();

    DataLog log = DataLogManager.getLog();

    DriverStation.startDataLog(DataLogManager.getLog());
  }

  private void configureBindings() {
    // set default commands
    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);
    SubsystemContainer.swerveSubsystem.setDefaultCommand(new SwerveDefaultCommand(m_driverController));
    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    SubsystemContainer.climbSubsystem.setDefaultCommand(m_climbDefaultCommand);
    SubsystemContainer.armSubsystem.setDefaultCommand(new ArmAngleCommand(Constants.ArmConstants.INTAKE_POSE));
    SubsystemContainer.shooterSubsystem.setDefaultCommand(new ShooterDefaultCommand());
    SubsystemContainer.limelightSubsystem.setDefaultCommand(m_LimelightDefaultCommand);

    // set named commands
    NamedCommands.registerCommand("MagicSpeakerCommand", m_magicSpeakerCommand);
    NamedCommands.registerCommand("IntakeCommand", m_intakeCommand);
    NamedCommands.registerCommand("IntakeDefaultCommand", m_intakeDefaultCommand);
    NamedCommands.registerCommand("AutoAlignCommand", m_autoAlignSequential);
    NamedCommands.registerCommand("ArmDefaultCommand", m_armDefaultCommand);
    NamedCommands.registerCommand("ArmAngleCommand", new ArmAngleCommand(ArmConstants.INTAKE_POSE));

    // Intake buttons
    m_driverController.leftBumper().whileTrue(new IntakeReverseCommand());
    m_driverController.leftTrigger().whileTrue(new IntakeCommand());

    // Climb buttons
    m_controller.rightStick().toggleOnTrue(new ClimbCommand(m_controller));

    // swerve buttons
    m_driverController.back().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));
    m_controller.leftTrigger().onTrue(new PathPlannerAuto("Two Note"));
    m_driverController.leftStick()
        .whileTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.setFastMode(true)));
    m_driverController.leftStick()
        .whileFalse(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.setFastMode(false)));

    m_driverController.a()
        .onTrue(new SequentialCommandGroup(SubsystemContainer.swerveSubsystem.m_pathfindAmp, new MagicAmpCommand()));

    m_driverController.b().onTrue(new InstantCommand(() -> {
      for (int i = 0; i < 4; i++) {
        SubsystemContainer.swerveSubsystem.swerveModules[i].resetEverything();
      }
    }));

    // shooter buttons
    m_controller.a().whileTrue(new MagicAmpCommand());
    m_controller.y().whileTrue(new MagicSpeakerCommand(m_angleCalculate));

    m_controller.x().whileTrue(
        new ShootManualCommand(Constants.ArmConstants.SUBWOOFER_ANGLE, Constants.ShooterSpeeds.SUBWOOFER_SPEED));

    m_controller.b()
        .whileTrue(new ShootManualCommand(Constants.ArmConstants.PODIUM_ANGLE, Constants.ShooterSpeeds.PODIUM_SPEED));
    m_driverController.rightBumper().whileTrue(new AutoAlignCommand());

    // arm buttons
    m_controller.leftStick().toggleOnTrue(new ArmManualCommand(m_controller));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void simulationInit() {
    SubsystemContainer.armSubsystem.simulationInit();
    SubsystemContainer.shooterSubsystem.simulationInit();
    SubsystemContainer.climbSubsystem.simulationInit();
  }

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public void teleopperiodic() {
  }
}
