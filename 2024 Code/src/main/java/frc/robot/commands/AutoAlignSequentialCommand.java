// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlignSequentialCommand extends AutoAlignCommand{
    double currentYaw;
    double targetYaw;
    private LimelightSubsystem m_limelight;
    private SwerveSubsystem m_swerveSubsystem;

    public AutoAlignSequentialCommand() {
        m_limelight = SubsystemContainer.limelightSubsystem;
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        addRequirements(m_limelight);
    }

    @Override
    public boolean isFinished() {
        if (m_swerveSubsystem.getCurrentAlliance() == true
                && m_limelight.getBotPose()[0] > -FieldConstants.COMMUNITYBOUNDARY) {
            return true;
        }
        if (m_swerveSubsystem.getCurrentAlliance() == false
                && m_limelight.getBotPose()[0] < FieldConstants.COMMUNITYBOUNDARY) {
            return true;
        }

        if (!m_swerveSubsystem.omegaZero()) {
            return true;
        }

        return false;
    }
}
