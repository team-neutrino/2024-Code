// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Gives the current yaw (test)
     */
    private int priorityTag;
    private XboxController m_xboxController;

    public AutoAlignCommand(CommandXboxController p_controller) {
        if (p_controller != null) {
            m_xboxController = p_controller.getHID();
        }
        addRequirements(SubsystemContainer.swerveSubsystem2);
    }

    @Override
    public void initialize() {
        // SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
        SubsystemContainer.limelightSubsystem.updateOdometryWithLimelightPose2();
        if (SubsystemContainer.alliance.isRedAlliance()) {
            priorityTag = AprilTagConstants.RED_ALLIANCE_IDS.SPEAKER_ID;
        } else {
            priorityTag = AprilTagConstants.BLUE_ALLIANCE_IDS.SPEAKER_ID;
        }
        SubsystemContainer.limelightSubsystem.setPriorityID(priorityTag);
    }

    @Override
    public void execute() {

        SubsystemContainer.swerveSubsystem2.setControl(drive
                .withVelocityX(m_xboxController.getLeftY() * SwerveConstants.MaxSpeed)
                .withVelocityY(m_xboxController.getLeftX() * SwerveConstants.MaxSpeed)
                .withRotationalRate(SubsystemContainer.limelightSubsystem.getTv()
                        ? convertOffsetAngleToOmega(SubsystemContainer.limelightSubsystem.getOffsetAngleFromTag())
                        : m_xboxController.getRightX() * SwerveConstants.MaxAngularRate));

        // SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);
    }

    /**
     * Helper method that converts the offset angle as retrieved by the limelight to
     * a rotational rate appropriate for autoaligning
     */
    private double convertOffsetAngleToOmega(double offsetAngle) {
        offsetAngle /= 29.8; // maximum possible tx value is 29.8 in either direction
        return offsetAngle * (SwerveConstants.MaxAngularRate * .75); // only use up to 75% turning power for autoalign
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}