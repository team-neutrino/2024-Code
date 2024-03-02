package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {
    private Pose2d botPose;
    private double[] botPoseArray;
    SwerveDrivePoseEstimator poseEstimator;

    double cycle = 0;

    public LimelightDefaultCommand() {
        poseEstimator = SubsystemContainer.swerveSubsystem.m_swervePoseEstimator;
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(0);
        SubsystemContainer.limelightSubsystem.setPriorityID(-1);
    }

    @Override
    public void execute() {
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            botPoseArray = SubsystemContainer.limelightSubsystem.getBotPose();
            if (SubsystemContainer.swerveSubsystem.isRedAlliance) {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw() + 180));
            } else {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw()));
            }

            cycle++;

            if (cycle % 14 == 0) {
                // double a;
                // if (SubsystemContainer.swerveSubsystem.isRedAlliance) {
                // a = Math.sqrt(Math.pow(botPose.getX() -
                // SwerveConstants.SPEAKER_RED_SIDE.getX(), 2)
                // + Math.pow(botPose.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY(), 2));
                // } else {
                // a = Math.sqrt(Math.pow(botPose.getX() -
                // SwerveConstants.SPEAKER_BLUE_SIDE.getX(), 2)
                // + Math.pow(botPose.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY(), 2));
                // }
                // System.out.println("distance to speaker " + a);

                System.out.println("x: -- " + botPose.getX());
                System.out.println("y: -- " + (botPose.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY()));
                // System.out.println("percent area " + botPoseArray[10]); returns percent as a
                // raw number, not less than 1
                // System.out.println("num of targets " + botPoseArray[7]);
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