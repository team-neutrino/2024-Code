// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {

    /**
     * Gives the current yaw (test)
     */
    protected double m_currentYaw;
    protected double m_offsetYaw;
    private int priorityTag;
    private double poseModifier;
    private XboxController m_xboxController;

    public AutoAlignCommand(CommandXboxController p_controller) {
        if (p_controller != null) {
            m_xboxController = p_controller.getHID();
        }
        addRequirements(SubsystemContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
        if (SubsystemContainer.alliance.isRedAlliance()) {
            priorityTag = AprilTagConstants.RED_ALLIANCE_IDS.SPEAKER_ID;
            poseModifier = 0;
        } else {
            priorityTag = AprilTagConstants.BLUE_ALLIANCE_IDS.SPEAKER_ID;
            poseModifier = 180;
        }
        SubsystemContainer.limelightSubsystem.setPriorityID(priorityTag);
    }

    @Override
    public void execute() {
        if (SubsystemContainer.limelightSubsystem.getTv()) {

            m_currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
            m_offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
            double[] pose = SubsystemContainer.limelightSubsystem.getBotPose();
            if (pose[5] > 0) {
                pose[5] -= poseModifier;
            } else {
                pose[5] += poseModifier;
            }

            if (SubsystemContainer.limelightSubsystem.getID() == (priorityTag)) {
                SubsystemContainer.swerveSubsystem
                        .setRobotYaw(SwerveSubsystem.calculateLimelightOffsetAngle(m_currentYaw, m_offsetYaw, pose[5]));
            }
        } else {
            // SUPER auto align!!
            SubsystemContainer.swerveSubsystem.AlignToSpeakerUsingOdometry();
        }

        SubsystemContainer.swerveSubsystem.SwerveWithDeadzone(m_xboxController.getLeftY() * -1,
                m_xboxController.getLeftX() * -1,
                m_xboxController.getRightX() * -1);

        SubsystemContainer.swerveSubsystem.POV(m_xboxController.getPOV());

        SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
