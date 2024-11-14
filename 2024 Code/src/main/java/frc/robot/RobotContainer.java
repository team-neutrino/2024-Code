// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> 79b8eef9 (commands updated to be explicit)
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

>>>>>>> f3f5dd4d (commands updated to be explicit)
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.GamePieceCommands.ArmClimbCommandDown;
import frc.robot.commands.GamePieceCommands.ArmClimbCommandUp;
import frc.robot.commands.GamePieceCommands.ArmManualCommand;
import frc.robot.commands.GamePieceCommands.AutonIntakeCommand;
import frc.robot.commands.GamePieceCommands.AutonShooterCommand;
import frc.robot.commands.GamePieceCommands.AutonSingleShotCommand;
import frc.robot.commands.GamePieceCommands.IntakeCommand;
import frc.robot.commands.GamePieceCommands.IntakeReverseCommand;
import frc.robot.commands.GamePieceCommands.MagicAmpChargeCommand;
import frc.robot.commands.GamePieceCommands.MagicShootCommand;
import frc.robot.commands.GamePieceCommands.MagicSpeakerChargeCommand;
import frc.robot.commands.GamePieceCommands.NoLLAutonShooterCommand;
import frc.robot.commands.GamePieceCommands.ShootManualCommand;
import frc.robot.commands.GamePieceCommands.ShootShuttleCommand;
import frc.robot.commands.GamePieceCommands.ShuttleCloseCommand;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignForeverCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.KrakenSwerveBrakeCommand;
import frc.robot.commands.KrakenSwerveDefaultCommand;
import frc.robot.commands.KrakenSwervePointCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShuttleAutoAlignCommand;
import frc.robot.util.SubsystemContainer;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

        SubsystemContainer m_subsystem_container = new SubsystemContainer();

        CommandXboxController m_buttonsController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
        CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

        LEDDefaultCommand m_LEDDefaultCommand = new LEDDefaultCommand(m_buttonsController, m_driverController);
        IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
        LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

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
                SubsystemContainer.swerveSubsystem2
                                .setDefaultCommand(new KrakenSwerveDefaultCommand(m_driverController));

                m_driverController.a().whileTrue(new KrakenSwerveBrakeCommand());
                m_driverController.b().whileTrue(new KrakenSwervePointCommand(m_driverController));

                m_driverController.back()
                                .onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem2.resetPigeon2()));

                if (Utils.isSimulation()) {
                        SubsystemContainer.swerveSubsystem2
                                        .seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                SubsystemContainer.swerveSubsystem2.registerTelemetry(logger::telemeterize);

                // set named commands
                NamedCommands.registerCommand("AutonIntakeCommand",
                                new AutonIntakeCommand(Constants.ArmConstants.INTAKE_POSE,
                                                Constants.ShooterSpeeds.INITIAL_SHOOTER_SPEED));
                NamedCommands.registerCommand("AutonShoot",
                                new NoLLAutonShooterCommand(Constants.ShooterSpeeds.SHOOTING_SPEED,
                                                SubsystemContainer.m_angleCalculate));
                NamedCommands.registerCommand("AutoAlignForever", new AutoAlignForeverCommand());
                NamedCommands.registerCommand("SingleSubwooferShot",
                                new AutonSingleShotCommand(ArmConstants.SUBWOOFER_ANGLE,
                                                Constants.ShooterSpeeds.SHOOTING_SPEED));

                // Intake buttons
                m_driverController.leftBumper().whileTrue(new IntakeReverseCommand());
                m_driverController.rightTrigger().whileTrue(new ShuttleCloseCommand());
                m_driverController.leftTrigger().whileTrue(new IntakeCommand());

                // shooter buttons

                m_buttonsController.a()
                                .whileTrue(new SequentialCommandGroup(new MagicAmpChargeCommand(m_buttonsController),
                                                new MagicShootCommand()));

                m_buttonsController.y().whileTrue(new SequentialCommandGroup(
                                new MagicSpeakerChargeCommand(m_buttonsController),
                                new MagicShootCommand()));

                m_buttonsController.x().whileTrue(new SequentialCommandGroup(
                                new ShootManualCommand(Constants.ArmConstants.SUBWOOFER_ANGLE,
                                                Constants.ShooterSpeeds.SHOOTING_SPEED,
                                                Constants.ShooterSpeeds.LOW_SPEED_THRESHOLD, m_buttonsController),
                                new MagicShootCommand()));

                m_buttonsController.b()
                                .whileTrue(new SequentialCommandGroup(
                                                new ShootShuttleCommand(Constants.ArmConstants.SHUTTLE_ANGLE,
                                                                m_buttonsController),
                                                new MagicShootCommand()));

                m_buttonsController.povDown()
                                .onTrue(new InstantCommand(
                                                () -> SubsystemContainer.armSubsystem.initializeMotorControllers()));

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
                        auto = new PathPlannerAuto("4 Note Amp No LL");
                } catch (Exception e) {
                        auto = new PathPlannerAuto("Nothing");
                }

                return auto;
        }

        public void teleopperiodic() {
        }
}
