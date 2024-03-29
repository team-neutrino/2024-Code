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
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
        timer.start();
    }

    @Override
    public void execute() {
        m_currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        m_offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
        pose = SubsystemContainer.limelightSubsystem.getBotPose();
        if (!SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            if (pose[5] > 0) {
                pose[5] -= 180;
            } else {
                pose[5] += 180;
            }

        }
        SubsystemContainer.swerveSubsystem
                .autonRotateSwerve(SwerveSubsystem.calculateLimelightOffsetAngle(m_currentYaw, m_offsetYaw, pose[5]));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs((SwerveSubsystem.calculateLimelightOffsetAngle(m_currentYaw, m_offsetYaw, pose[5]))
                - (m_currentYaw)) < 0.5) {
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
