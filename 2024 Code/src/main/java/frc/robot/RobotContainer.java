// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.MagicAmpCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ArmAngleCommand;
import frc.robot.commands.ArmInterpolateCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.util.SubsystemContainer;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.CalculateAngle;

public class RobotContainer {

  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_controller = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_controller);
  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand();
  IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
  ClimbDefaultCommand m_climbDefaultCommand = new ClimbDefaultCommand();
  LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();
  CalculateAngle m_angleCalculate = new CalculateAngle();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // set default commands
    SubsystemContainer.LEDSubsystem.setDefaultCommand(m_LEDDefaultCommand);
    SubsystemContainer.swerveSubsystem.setDefaultCommand(m_swerveDefaultCommand);
    SubsystemContainer.intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    SubsystemContainer.climbSubsystem.setDefaultCommand(m_climbDefaultCommand);
    SubsystemContainer.armSubsystem.setDefaultCommand(new ArmInterpolateCommand(m_angleCalculate));
    SubsystemContainer.ShooterSubsystem.setDefaultCommand(new ShooterDefaultCommand());
    SubsystemContainer.limelightSubsystem.setDefaultCommand(m_LimelightDefaultCommand);

    // LED buttons
    m_controller.a().whileTrue(new MagicAmpCommand());

    // Intake buttons
    m_controller.leftBumper().whileTrue(new IntakeReverseCommand());

    // Climb buttons
    m_controller.rightStick().toggleOnTrue(new ClimbCommand(m_controller));

    // swerve buttons
    m_driverController.b().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetNavX()));
    m_driverController.leftBumper().onTrue(new PathPlannerAuto("New Auto"));

    // shooter buttons
    m_controller.y().whileTrue(new ShootSpeakerCommand());
    m_controller.rightBumper().whileTrue(new AutoAlignCommand());

    // arm buttons
    m_controller.leftStick().toggleOnTrue(new ArmManualCommand(m_controller));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void simulationInit() {
    SubsystemContainer.ShooterSubsystem.simulationInit();
  }

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public void periodic() {
  }
}
