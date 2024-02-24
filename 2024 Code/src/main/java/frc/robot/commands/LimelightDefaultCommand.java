package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {
    private SwerveSubsystem m_swerveSubsystem;
    private LimelightSubsystem m_limelightSubsystem;
    private Pose2d botPose;
    private double[] botPoseArray;
    private double poseDifference;
    SwerveDrivePoseEstimator poseEstimator;

    double cycle = 0;

    public LimelightDefaultCommand() {
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        m_limelightSubsystem = SubsystemContainer.limelightSubsystem;
        poseEstimator = m_swerveSubsystem.m_swervePoseEstimator;
        addRequirements(m_limelightSubsystem);
    }

    @Override
    public void initialize() {
        m_limelightSubsystem.setPipeline(0);
    }

    @Override
    public void execute() {
        if (m_limelightSubsystem.getTv()) {
            botPoseArray = m_limelightSubsystem.getBotPose();
            botPose = new Pose2d(botPoseArray[0],
                    botPoseArray[1],
                    Rotation2d.fromDegrees(m_swerveSubsystem.getYaw()));

            // //invalid limelight data
            // if (botPose.getX() != 0.0)
            // {
            // poseDifference =
            // poseEstimator.getEstimatedPosition().getTranslation().getDistance(botPose.getTranslation());
            // }

            if (Math.sqrt(Math.pow(botPose.getX() - m_swerveSubsystem.currentPoseL.getX(), 2)
                    + Math.pow(botPose.getY() - m_swerveSubsystem.currentPoseL.getY(), 2)) > 1.5) {
                m_swerveSubsystem.resetPose(botPose);
            }

            cycle++;

            if (cycle % 8 ==0)
            {
                double a;
                if (m_swerveSubsystem.isRedAlliance)
                {
                    a = Math.sqrt(Math.pow(botPose.getX() - SwerveConstants.SPEAKER_RED_SIDE.getX(), 2) + Math.pow(botPose.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY(), 2));
                }
                else
                {
                    a = Math.sqrt(Math.pow(botPose.getX() - SwerveConstants.SPEAKER_BLUE_SIDE.getX(), 2) + Math.pow(botPose.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY(), 2));
                }
                System.out.println("distance to speaker " + a);
            }

            m_swerveSubsystem.m_swervePoseEstimator.addVisionMeasurement(botPose,
                    Timer.getFPGATimestamp() - (botPoseArray[6] / 1000.0));
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