// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.AmpAutoAlign;
import frc.robot.commands.GamePieceCommands.ArmClimbCommandDown;
import frc.robot.commands.GamePieceCommands.ArmClimbCommandUp;
import frc.robot.commands.GamePieceCommands.ArmManualCommand;
import frc.robot.commands.GamePieceCommands.AutonIntakeCommand;
import frc.robot.commands.GamePieceCommands.AutonShooterCommand;
import frc.robot.commands.GamePieceCommands.AutonSingleShotCommand;
import frc.robot.commands.GamePieceCommands.IntakeCommand;
import frc.robot.commands.GamePieceCommands.IntakeReverseCommand;
import frc.robot.commands.GamePieceCommands.MagicAmpChargeCommand;
import frc.robot.commands.GamePieceCommands.MagicERAmpChargeCommand;
import frc.robot.commands.GamePieceCommands.MagicShootCommand;
import frc.robot.commands.GamePieceCommands.MagicSpeakerChargeCommand;
import frc.robot.commands.GamePieceCommands.ShootManualCommand;
import frc.robot.commands.GamePieceCommands.ShootShuttleCommand;
import frc.robot.commands.GamePieceCommands.ShuttleCloseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignForeverCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShuttleAutoAlignCommand;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.TunerConstants;

// import org.jcp.xml.dsig.internal.dom.Utils;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_buttonsController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand(m_buttonsController, m_driverController);
  IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
  LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain m_swerve = TunerConstants.DriveTrain; // My m_swerve

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

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
    SubsystemContainer.limelightSubsystem.setDefaultCommand(m_LimelightDefaultCommand);
    m_swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        m_swerve.applyRequest(() -> drive.withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive forward
                                                                                                  // with
            // negative Y (forward)
            .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
        ));

    m_driverController.a().whileTrue(m_swerve.applyRequest(() -> brake));
    m_driverController.b().whileTrue(m_swerve
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldRelative()));

    if (Utils.isSimulation()) {
      m_swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_swerve.registerTelemetry(logger::telemeterize);

    // set named commands
    NamedCommands.registerCommand("AutonIntakeCommand",
        new AutonIntakeCommand(Constants.ArmConstants.INTAKE_POSE, Constants.ShooterSpeeds.INITIAL_SHOOTER_SPEED));
    NamedCommands.registerCommand("AutonShoot",
        new AutonShooterCommand(Constants.ShooterSpeeds.SHOOTING_SPEED, SubsystemContainer.m_angleCalculate));
    NamedCommands.registerCommand("AutoAlignForever", new AutoAlignForeverCommand());
    NamedCommands.registerCommand("SingleSubwooferShot",
        new AutonSingleShotCommand(ArmConstants.SUBWOOFER_ANGLE, Constants.ShooterSpeeds.SHOOTING_SPEED));

    // Intake buttons
    m_driverController.leftBumper().whileTrue(new IntakeReverseCommand());
    m_driverController.rightTrigger().whileTrue(new ShuttleCloseCommand());
    m_driverController.leftTrigger().whileTrue(new IntakeCommand());

    // swerve buttons
    m_driverController.back().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetPigeon2()));

    // m_driverController.b().onTrue(new InstantCommand(() -> {
    // SubsystemContainer.swerveSubsystem.ResetModules();
    // SubsystemContainer.armSubsystem.initializeMotorControllers();
    // }));

    // m_driverController.x().whileTrue(new AmpAutoAlign(m_driverController));

    // shooter buttons

    m_buttonsController.a()
        .whileTrue(new SequentialCommandGroup(new MagicAmpChargeCommand(m_buttonsController), new MagicShootCommand()));

    m_driverController.start()
        .whileTrue(new InstantCommand(() -> SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose()));

    m_buttonsController.y().whileTrue(new SequentialCommandGroup(
        new MagicSpeakerChargeCommand(m_buttonsController),
        new MagicShootCommand()));

    m_buttonsController.x().whileTrue(new SequentialCommandGroup(
        new ShootManualCommand(Constants.ArmConstants.SUBWOOFER_ANGLE, Constants.ShooterSpeeds.SHOOTING_SPEED,
            Constants.ShooterSpeeds.LOW_SPEED_THRESHOLD, m_buttonsController),
        new MagicShootCommand()));

    m_buttonsController.b()
        .whileTrue(new SequentialCommandGroup(new ShootShuttleCommand(Constants.ArmConstants.SHUTTLE_ANGLE,
            m_buttonsController), new MagicShootCommand()));

    m_buttonsController.povDown()
        .onTrue(new InstantCommand(() -> SubsystemContainer.armSubsystem.initializeMotorControllers()));

    m_driverController.rightBumper().whileTrue(new AutoAlignCommand(m_driverController));

    m_driverController.y().whileTrue(new ShuttleAutoAlignCommand(m_buttonsController));

    // arm buttons
    m_buttonsController.leftStick().toggleOnTrue(new ArmManualCommand(m_buttonsController));
    m_buttonsController.back().toggleOnTrue(new ArmClimbCommandDown());
    m_buttonsController.start().toggleOnTrue(new ArmClimbCommandUp());
  }

  public Command getAutonomousCommand() {
    Command auto;
    try {
      auto = new PathPlannerAuto("4 Note AMP");
    } catch (Exception e) {
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }

  public void teleopperiodic() {
  }
}
