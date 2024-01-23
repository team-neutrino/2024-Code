package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {

    Pose2d pose;
    double[] botPose;
    private LimelightSubsystem m_limelight;
    private SwerveSubsystem m_swerveSubsystem;

    public LimelightDefaultCommand() {
        m_limelight = SubsystemContainer.limelightSubsystem;
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        addRequirements(m_limelight);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_limelight.getTv()) {
            botPose = m_limelight.getBotPose();
            pose = new Pose2d(botPose[0], botPose[1],
                    Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw()));
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
