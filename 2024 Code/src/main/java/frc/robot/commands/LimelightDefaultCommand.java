package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {
    private Pose2d botPose;
    private double[] botPoseArray;
    SwerveDrivePoseEstimator poseEstimator;

    public LimelightDefaultCommand() {
        poseEstimator = SubsystemContainer.swerveSubsystem.m_swervePoseEstimator;
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(0);
        if (SubsystemContainer.alliance.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}