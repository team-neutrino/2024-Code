package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;

public class LimelightDefaultCommand extends Command {
    private Pose2d pose;
    private double[] botPose;

    public LimelightDefaultCommand() {
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
            botPose = SubsystemContainer.limelightSubsystem.getBotPose();
            pose = new Pose2d(botPose[0] + SwerveConstants.CENTER_OF_FIELD_M.getX(),
                    botPose[1] + SwerveConstants.CENTER_OF_FIELD_M.getY(),
                    Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem.getYaw()));
            SubsystemContainer.swerveSubsystem.resetPose(pose);

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
