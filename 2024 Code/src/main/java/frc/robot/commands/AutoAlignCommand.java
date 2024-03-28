// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {

    /**
     * Gives the current yaw (test)
     */
    protected double currentYaw;

    protected double offsetYaw;

    // Why not protected?
    // Why does it even exist? Can't it just be a local?
    double y = 0;
    double x = 0;

    public AutoAlignCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
        // We are bad about this whole add requirements thing
        addRequirements(SubsystemContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Why is the swerve responsible for storing our alliance?
        if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }

        // Do we really only intend to update the odometry at the start of alignment and not during
        // the entire alignment?
        SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
    }

    @Override
    public void execute() {
        // What? What is Tv, why do we care?
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
            // Tx is not equivalent to yaw. Why not ask the limelight for its best estimate of the yaw relative to the robot pose?
            // This is probably why we oscillate when aligning sometimes
            offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
            // Give some kind of name to pose[5].
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
            // Minor improvement, set the constant to a local and do the math the same after. Will reduce duplicated code
            if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
                // Why is it called currentPoseL?
                // This seems backwards. Don't we want the yaw from our robot to the speaker and not from the speaker to the robot?
                // A vector from A->B is calculated via B-A not A-B
                // IS THIS WHY WE SPIN LIKE AN IDIOT IF WE MISS THE TAG?!
                y = SubsystemContainer.swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY();
                x = SubsystemContainer.swerveSubsystem.currentPoseL.getX() - SwerveConstants.SPEAKER_RED_SIDE.getX();
            } else {
                y = SubsystemContainer.swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY();
                x = SubsystemContainer.swerveSubsystem.currentPoseL.getX() - SwerveConstants.SPEAKER_BLUE_SIDE.getX();
            }
            // Bold to use atan and not atan2
            SubsystemContainer.swerveSubsystem.setRobotYaw(Math.toDegrees(Math.atan(y / x)));
        }
        // Should we set this at the start? I'm not really sure how the lock system works
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
