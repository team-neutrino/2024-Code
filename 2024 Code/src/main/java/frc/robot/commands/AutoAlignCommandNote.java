// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.PhotonVision;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class AutoAlignCommandNote extends Command {
    Transform3d currentInfo;
    double currentYaw;
    double targetYaw;

    public AutoAlignCommandNote() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(1);
    }

    @Override
    public void execute() {
        if (SubsystemContainer.photonVision.hasTarget()) {
            currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
            targetYaw = PhotonVision.getYaw();
            SubsystemContainer.swerveSubsystem.setRobotYaw(currentYaw - targetYaw);
            // SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);
            // double referenceAngle = SubsystemContainer.photonVision.getYaw();
            // double yaw = SubsystemContainer.swerveSubsystem.getYaw();
            // double distanceFromTarget = PhotonUtils.calculateDistanceToTargetMeters(
            // Constants.PhotonVisionConstants.CAMERA_HEIGHT_METERS,
            // Constants.PhotonVisionConstants.TARGET_HEIGHT_METERS,
            // Constants.PhotonVisionConstants.CAMERA_PITCH_RADIANS,
            // Units.degreesToRadians(SubsystemContainer.photonVision.getResult().getBestTarget().getPitch()));
            // // double vy = Math.sin(yaw) * distanceFromTarget;
            // // double vx = Math.cos(yaw) * distanceFromTarget;
            // double vy = 0;
            // double vx = 0;
            // System.out.println(vy + " " + vx + " " + yaw + " " + distanceFromTarget);
            // SubsystemContainer.swerveSubsystem.SwerveWithoutDeadzone(vx, vy,
            // (referenceAngle - yaw) / 57.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
