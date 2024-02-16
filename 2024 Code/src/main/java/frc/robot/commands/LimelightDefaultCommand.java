package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {
    private SwerveSubsystem m_swerveSubsystem;
    private LimelightSubsystem m_limelightSubsystem;
    private Pose2d pose;
    private double[] botPose;

    public LimelightDefaultCommand() {
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        m_limelightSubsystem = SubsystemContainer.limelightSubsystem;
        addRequirements(m_limelightSubsystem);

    }

    @Override
    public void initialize() {
        m_limelightSubsystem.setPipeline(0);
    }

    @Override
    public void execute() {
        if (m_limelightSubsystem.getTv()) {
            botPose = m_limelightSubsystem.getBotPose();
            pose = new Pose2d(botPose[0], botPose[1],
                    Rotation2d.fromDegrees(m_swerveSubsystem.getYaw()));
            m_swerveSubsystem.resetPose(pose);

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
