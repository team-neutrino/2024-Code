package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {

    Pose2d position;
    double[] pose;
    private LimelightSubsystem m_LimelightSubsystem;

    public LimelightDefaultCommand() {
        m_LimelightSubsystem = SubsystemContainer.LimelightSubsystem;
        addRequirements(m_LimelightSubsystem);

    }

    @Override
    public void initialize() {
        if (m_LimelightSubsystem.getTv()) {
            pose = m_LimelightSubsystem.getBotPose();
            position = new Pose2d(pose[0], pose[2], new Rotation2d(pose[5]));
        }

    }

    @Override
    public void execute() {
        if (m_LimelightSubsystem.getTv()) {
            pose = m_LimelightSubsystem.getBotPose();
            position = new Pose2d(pose[0], pose[2], new Rotation2d(pose[5]));
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
