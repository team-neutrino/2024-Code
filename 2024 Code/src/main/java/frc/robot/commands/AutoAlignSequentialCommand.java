// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldConstants;
import frc.robot.util.SubsystemContainer;

public class AutoAlignSequentialCommand extends AutoAlignCommand {
    double currentYaw;
    double targetYaw;

    public AutoAlignSequentialCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public boolean isFinished() {
        if (SubsystemContainer.swerveSubsystem.getCurrentAlliance() == true
                && SubsystemContainer.limelightSubsystem.getBotPose()[0] > -FieldConstants.COMMUNITYBOUNDARY) {
            return true;
        }
        if (SubsystemContainer.swerveSubsystem.getCurrentAlliance() == false
                && SubsystemContainer.limelightSubsystem.getBotPose()[0] < FieldConstants.COMMUNITYBOUNDARY) {
            return true;
        }

        if (!SubsystemContainer.swerveSubsystem.omegaZero()) {
            return true;
        }

        return false;
    }
}
