// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.PhotonVision;

/** An example command that uses an example subsystem. */
public class AutoAlignCommandNote extends Command {
    Transform3d currentInfo;
    double currentYaw;
    double targetYaw;

    public AutoAlignCommandNote() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(1);
    }

    @Override
    public void execute() {
        currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        targetYaw = PhotonVision.getYaw();
        SubsystemContainer.swerveSubsystem.setRobotYaw(currentYaw - targetYaw);
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
