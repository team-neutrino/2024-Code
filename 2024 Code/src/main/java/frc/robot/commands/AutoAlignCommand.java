// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import frc.robot.util.SwerveRequestStash;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {

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
        SubsystemContainer.swerveSubsystem2.setControl(SwerveRequestStash.drive
                .withVelocityX(m_xboxController.getLeftY() * SwerveConstants.MaxSpeed)
                .withVelocityY(m_xboxController.getLeftX() * SwerveConstants.MaxSpeed)
                .withRotationalRate(
                        offsetToOmega(-SubsystemContainer.limelightSubsystem.getOffsetAngleFromTag())));

        // SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);
    }

    /**
     * Helper method that converts the offset angle as retrieved by the limelight to
     * a rotational rate appropriate for autoaligning. Uses proportional control.
     */
    private double offsetToOmega(double offsetAngle) {
        offsetAngle /= 32; // maximum possible tx value is 29.8 in either direc

        double scaler = SwerveConstants.MaxAngularRate * .5;

        System.out.println(offsetAngle * scaler);
        // Use power proportional to the offset angle
        return offsetAngle * scaler;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}