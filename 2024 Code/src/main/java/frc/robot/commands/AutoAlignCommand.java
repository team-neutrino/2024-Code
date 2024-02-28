// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LEDConstants.States;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {
    private double currentYaw;
    private double offsetYaw;

    double y = 0;
    double x = 0;
    
    public AutoAlignCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
    }

    @Override
    public void execute() {
        if (m_limelightSubsystem.getTv())
        {
            currentYaw = m_swerveSubsystem.getYaw();
            offsetYaw = m_limelightSubsystem.getTx();
            m_swerveSubsystem.setRobotYaw(currentYaw - offsetYaw);
        }
        else
        {
            //SUPER auto align!!
            if (m_swerveSubsystem.isRedAlliance)
            {
                y = m_swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY();
                x = m_swerveSubsystem.currentPoseL.getX() - SwerveConstants.SPEAKER_RED_SIDE.getX();
            }
            else
            {
                y = m_swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY();
                x = m_swerveSubsystem.currentPoseL.getX() - SwerveConstants.SPEAKER_BLUE_SIDE.getX();
            }

            m_swerveSubsystem.setRobotYaw(Math.toDegrees(Math.atan(y / x)));
        }
        m_swerveSubsystem.setCommandState(States.AUTOALIGN);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
