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

    public AutoAlignCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
        SwerveSubsystem.autoAlign = true;
    }

    @Override
    public void execute() {
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            // m_currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
            // m_offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
            // double[] pose = SubsystemContainer.limelightSubsystem.getBotPose();
            // if (!SubsystemContainer.alliance.isRedAlliance()) {
            // if (pose[5] > 0) {
            // pose[5] -= 180;
            // } else {
            // pose[5] += 180;
            // }
            // }
            SubsystemContainer.swerveSubsystem
                    .setRobotYaw(SwerveSubsystem.calculateLimelightOffsetAngle());

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
