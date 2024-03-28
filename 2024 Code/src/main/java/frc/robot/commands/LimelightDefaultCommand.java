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
        // Grabbing a variable from a subsystem counts as using it
        addRequirements(SubsystemContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(0);
        // Copied code again
        if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
    }

    @Override
    public void execute() {
        // Big block of copied code, same bizarre stuff with Tv
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            botPoseArray = SubsystemContainer.limelightSubsystem.getBotPose();
            if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw() + 180));
            } else {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw()));
            }

            SubsystemContainer.limelightSubsystem.updatePoseEstimatorWithVisionBotPose(
                    SubsystemContainer.swerveSubsystem.m_swervePoseEstimator, botPose);

        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}