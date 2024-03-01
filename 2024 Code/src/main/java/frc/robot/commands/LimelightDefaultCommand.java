package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
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
        SubsystemContainer.limelightSubsystem.setPriorityID(-1);
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        if (m_limelightSubsystem.getTv()) {
            botPoseArray = m_limelightSubsystem.getBotPose();
            if (m_swerveSubsystem.isRedAlliance)
            {
                //this will be needed unless the limelight knows that it is mounted backwards I think
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(m_swerveSubsystem.getYaw() + 180));
            }
            else
            {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(m_swerveSubsystem.getYaw()));
            }
            
            

            // //invalid limelight data
            // if (botPose.getX() != 0.0)
            // {
            // poseDifference =
            // poseEstimator.getEstimatedPosition().getTranslation().getDistance(botPose.getTranslation());
            // }

            // if (Math.sqrt(Math.pow(botPose.getX() -
            // m_swerveSubsystem.currentPoseL.getX(), 2)
            // + Math.pow(botPose.getY() - m_swerveSubsystem.currentPoseL.getY(), 2)) > 1.5)
            // {
            // m_swerveSubsystem.resetPose(botPose);
            // }

            // cycle++;

            // if (cycle % 8 == 0) {
            // double a;
            // if (m_swerveSubsystem.isRedAlliance) {
            // a = Math.sqrt(Math.pow(botPose.getX() -
            // SwerveConstants.SPEAKER_RED_SIDE.getX(), 2)
            // + Math.pow(botPose.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY(), 2));
            // } else {
            // a = Math.sqrt(Math.pow(botPose.getX() -
            // SwerveConstants.SPEAKER_BLUE_SIDE.getX(), 2)
            // + Math.pow(botPose.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY(), 2));
            // }
            // System.out.println("distance to speaker " + a);
            // }

            m_swerveSubsystem.m_swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(4.0, 4.0, 180.0));

            m_swerveSubsystem.m_swervePoseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp() - (botPoseArray[6] / 1000.0));
=======
        if (SubsystemContainer.limelightSubsystem.getTv()) {
            botPoseArray = SubsystemContainer.limelightSubsystem.getBotPose();
            if (SubsystemContainer.swerveSubsystem.isRedAlliance) {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw() + 180));
            } else {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw()));
            }

            SubsystemContainer.limelightSubsystem.updatePoseEstimatorWithVisionBotPose(
                    SubsystemContainer.swerveSubsystem.m_swervePoseEstimator, botPose);

>>>>>>> main
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