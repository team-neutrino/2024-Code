// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutoAlignSequentialCommand extends AutoAlignCommand {
    private SwerveSubsystem m_swerveSubsystem;
    private LimelightSubsystem m_limelightSubsystem;
    private double currentYaw;
    private double targetYaw;
    private Timer timer;

    public AutoAlignSequentialCommand() {
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        m_limelightSubsystem = SubsystemContainer.limelightSubsystem;
        timer = new Timer();
        addRequirements(m_limelightSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        m_swerveSubsystem.Swerve(0, 0, 0);
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

        if (timer.get() >= 3) {
            timer.stop();
            timer.reset();
            return true;
        }

        return false;
    }
}
