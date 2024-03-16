// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.SubsystemContainer;

public class AutoAlignSequentialCommand extends AutoAlignCommand {

    Timer timer = new Timer();
    double[] pose;

    public AutoAlignSequentialCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
    }

    @Override
    public void execute() {
        currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
        pose = SubsystemContainer.limelightSubsystem.getBotPose();
        System.out.println("tx " + offsetYaw);
        SubsystemContainer.swerveSubsystem.autonRotateSwerve(currentYaw - offsetYaw + (pose[5] * 0.04));

    }

    @Override
    public boolean isFinished() {
        System.out.println("termination - " + (Math.abs((currentYaw - offsetYaw + (pose[5] * 0.04)) - (currentYaw))));
        if (Math.abs((currentYaw - offsetYaw + (pose[5] * 0.04)) - (currentYaw)) < 0.5) {
            SubsystemContainer.swerveSubsystem.stopSwerve();
            return true;
        }
        // } else if (Math.abs(startYaw - currentYaw) > 6) {
        // return true;
        // } else if (timer.get() > 2) {
        // timer.stop();
        // timer.reset();
        // return true;
        // }

        return false;
    }
}
