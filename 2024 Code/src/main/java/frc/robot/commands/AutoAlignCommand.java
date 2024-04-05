// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {

    /**
     * Gives the current yaw (test)
     */
    protected double m_currentYaw;
    protected double m_offsetYaw;
    private boolean priorityTag;

    public AutoAlignCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
        priorityTag = false;
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
        if (SubsystemContainer.alliance.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
    }

    @Override
    public void execute() {
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            if (SubsystemContainer.alliance.isRedAlliance()) {
                if (SubsystemContainer.limelightSubsystem.getID().equals(4.0)) {
                    priorityTag = true;
                } else {
                    priorityTag = false;
                }
            } else {
                if (SubsystemContainer.limelightSubsystem.getID().equals(4.0)) {
                    priorityTag = true;
                } else {
                    priorityTag = false;
                }
            }

            m_currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
            m_offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
            double[] pose = SubsystemContainer.limelightSubsystem.getBotPose();
            if (!SubsystemContainer.alliance.isRedAlliance()) {
                if (pose[5] > 0) {
                    pose[5] -= 180;
                } else {
                    pose[5] += 180;
                }
            }

            if (priorityTag) {
                SubsystemContainer.swerveSubsystem
                        .setRobotYaw(SwerveSubsystem.calculateLimelightOffsetAngle(m_currentYaw, m_offsetYaw, pose[5]));

            }
        } else {
            // SUPER auto align!!
            SubsystemContainer.swerveSubsystem.AlignToSpeakerUsingOdometry();
        }

        SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
