// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutoAlignForeverCommand extends AutoAlignCommand {

    Timer timer = new Timer();
    double[] pose;

    public AutoAlignForeverCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        if (SubsystemContainer.alliance.isRedAlliance()) {
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
        if (!SubsystemContainer.alliance.isRedAlliance()) {
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
        return false;
    }
}
