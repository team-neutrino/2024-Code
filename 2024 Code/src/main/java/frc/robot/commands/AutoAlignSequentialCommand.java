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
        super.execute();
        SubsystemContainer.swerveSubsystem.Swerve(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        // if (m_swerveSubsystem.isRedAlliance() == true
        // && m_limelightSubsystem.getBotPose()[0] > -FieldConstants.COMMUNITYBOUNDARY)
        // {
        // return true;
        // }
        // if (m_swerveSubsystem.isRedAlliance() == false
        // && m_limelightSubsystem.getBotPose()[0] < FieldConstants.COMMUNITYBOUNDARY) {
        // return true;
        // }

        // if (!m_swerveSubsystem.omegaZero()) {
        // return true;
        // }

        if (timer.get() >= 1) {
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
    }
}
