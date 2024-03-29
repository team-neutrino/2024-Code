// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {

    /**
     * Gives the current yaw (test)
     */
    private double currentYaw;
    private double offsetYaw;

    private double y = 0;
    private double x = 0;

    private Translation2d m_speakerPose;

    public AutoAlignCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
            m_speakerPose = SwerveConstants.SPEAKER_RED_SIDE;
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
            m_speakerPose = SwerveConstants.SPEAKER_BLUE_SIDE;
        }

        SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
    }

    @Override
    public void execute() {
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
            offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
            double[] pose = SubsystemContainer.limelightSubsystem.getBotPose();
            if (!SubsystemContainer.swerveSubsystem.isRedAlliance()) {
                if (pose[5] > 0) {
                    pose[5] -= 180;
                } else {
                    pose[5] += 180;
                }
            }
            SubsystemContainer.swerveSubsystem
                    .setRobotYaw(SwerveSubsystem.calculateLimelightOffsetAngle(currentYaw, offsetYaw, pose[5]));

        } else {
            // SUPER auto align!!
            y = m_speakerPose.getY() - SubsystemContainer.swerveSubsystem.currentPoseL.getY();
            x = m_speakerPose.getX() - SubsystemContainer.swerveSubsystem.currentPoseL.getX();

            SubsystemContainer.swerveSubsystem.setRobotYaw(Math.toDegrees(Math.atan2(y, x)));
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
