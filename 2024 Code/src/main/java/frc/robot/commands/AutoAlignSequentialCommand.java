// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutoAlignSequentialCommand extends AutoAlignCommand {
    private SwerveSubsystem m_swerveSubsystem;
    private LimelightSubsystem m_limelightSubsystem;
    private double currentYaw;
    private double targetYaw;

    public AutoAlignSequentialCommand() {
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        m_limelightSubsystem = SubsystemContainer.limelightSubsystem;
        addRequirements(m_limelightSubsystem);
    }

    @Override
    public boolean isFinished() {
        if (m_swerveSubsystem.isRedAlliance() == true
                && m_limelightSubsystem.getBotPose()[0] > -FieldConstants.COMMUNITYBOUNDARY) {
            return true;
        }
        if (m_swerveSubsystem.isRedAlliance() == false
                && m_limelightSubsystem.getBotPose()[0] < FieldConstants.COMMUNITYBOUNDARY) {
            return true;
        }

        if (!m_swerveSubsystem.omegaZero()) {
            return true;
        }

        return false;
    }
}
