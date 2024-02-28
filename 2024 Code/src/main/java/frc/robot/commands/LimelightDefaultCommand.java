package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {
    private SwerveSubsystem m_swerveSubsystem;
    private LimelightSubsystem m_limelightSubsystem;
    private Pose2d botPose;
    private double[] botPoseArray;
    SwerveDrivePoseEstimator poseEstimator;

    public LimelightDefaultCommand() {
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        m_limelightSubsystem = SubsystemContainer.limelightSubsystem;
        poseEstimator = m_swerveSubsystem.m_swervePoseEstimator;
        addRequirements(m_limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(0);
        SubsystemContainer.limelightSubsystem.setPriorityID(-1);
    }

    @Override
    public void execute() {
        if (m_limelightSubsystem.getTv()) {
            botPoseArray = m_limelightSubsystem.getBotPose();
            if (m_swerveSubsystem.isRedAlliance) {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(m_swerveSubsystem.getYaw() + 180));
            } else {
                botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
                        Rotation2d.fromDegrees(m_swerveSubsystem.getYaw()));
            }

            m_limelightSubsystem.updatePoseEstimatorWithVisionBotPose(m_swerveSubsystem.m_swervePoseEstimator, botPose);
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