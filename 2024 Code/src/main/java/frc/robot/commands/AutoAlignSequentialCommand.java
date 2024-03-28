// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutoAlignSequentialCommand extends AutoAlignCommand {

    Timer timer = new Timer();
    double[] pose;

    public AutoAlignSequentialCommand() {
         // Both of these requirement are added in the parent, is that bad? Does it cause issues?
        addRequirements(SubsystemContainer.limelightSubsystem);
        // We are bad about this whole add requirements thing
        addRequirements(SubsystemContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        // This is already done in the parent
        if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
        timer.start();
    }

    @Override
    public void execute() {
        // Too much copied code from the autoaligncommand. Pull out the function and share it.
        // Also, stop using Tx. See comments in AutoAlignCommand
        currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
        pose = SubsystemContainer.limelightSubsystem.getBotPose();
        if (!SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            if (pose[5] > 0) {
                pose[5] -= 180;
            } else {
                pose[5] += 180;
            }

        }
        SubsystemContainer.swerveSubsystem
                .autonRotateSwerve(SwerveSubsystem.calculateLimelightOffsetAngle(currentYaw, offsetYaw, pose[5]));
    }

    @Override
    public boolean isFinished() {
        // Why not ask the subsystem for its newest yaw? Are we guaranteed to have updated currentYaw to the latest value by the time we get here?
        if (Math.abs((SwerveSubsystem.calculateLimelightOffsetAngle(currentYaw, offsetYaw, pose[5]))
                - (currentYaw)) < 0.5) {
            // Why do we stop swerve?
            SubsystemContainer.swerveSubsystem.stopSwerve();
            timer.stop();
            timer.reset();
            return true;
        } else if (timer.get() > 1.5) {
            SubsystemContainer.swerveSubsystem.stopSwerve();
            timer.stop();
            timer.reset();
            return true;
        }

        return false;
    }
}
