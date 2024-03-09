// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.util.SubsystemContainer;

public class AutoAlignSequentialCommand extends AutoAlignCommand {

    Timer timer = new Timer();

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
        // super.execute();
        // SubsystemContainer.swerveSubsystem.Swerve(0, 0, 0);
        currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
        SubsystemContainer.swerveSubsystem.autonRotateSwerve(currentYaw - offsetYaw);
        System.out.println("Current yaw: " + currentYaw + "\nOffset yaw " + offsetYaw);

    }

    @Override
    public boolean isFinished() {
        // if (timer.get() >= 2) {
        // timer.stop();
        // timer.reset();
        // return true;
        // }
        // return false;

        if (SubsystemContainer.limelightSubsystem.getTx() < 1) {
            return true;
        }

        return false;
    }
}
