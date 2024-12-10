// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignCommandNote;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.PhotonVisionDefaultCommand;
import frc.robot.util.SubsystemContainer;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  SubsystemContainer m_subsystem_container = new SubsystemContainer();

  CommandXboxController m_buttonsController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER);
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  LimelightDefaultCommand m_LimelightDefaultCommand = new LimelightDefaultCommand();
  PhotonVisionDefaultCommand m_PhotonVisionDefaultCommand = new PhotonVisionDefaultCommand();

  public RobotContainer() {
    configureBindings();

    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());
  }

  private void configureBindings() {
    // set default commands
    SubsystemContainer.swerveSubsystem.setDefaultCommand(new SwerveDefaultCommand(m_driverController));
    SubsystemContainer.limelightSubsystem.setDefaultCommand(m_LimelightDefaultCommand);
    SubsystemContainer.photonVision.setDefaultCommand(m_PhotonVisionDefaultCommand);

    // set named commands
    // Intake buttons

    // swerve buttons
    m_driverController.back().onTrue(new InstantCommand(() -> SubsystemContainer.swerveSubsystem.resetPigeon2()));

    // shooter buttons

    m_driverController.start()
        .whileTrue(new InstantCommand(() -> SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose()));

    // m_driverController.a().whileTrue(new InstantCommand(() ->
    // SubsystemContainer.m_angleCalculate.dumpData()));

    // separate button binding to left bumper contained within the magic speaker
    // charge command

    m_driverController.rightBumper().whileTrue(new AutoAlignCommand(m_driverController));

    // arm buttons

    // photonvision stuff
    m_driverController.rightTrigger().whileTrue(new AutoAlignCommandNote());
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
