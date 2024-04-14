// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutoAlignForeverCommand extends AutoAlignCommand {

    Timer timer = new Timer();
    double[] pose;

    public AutoAlignForeverCommand() {
        super(null);
        addRequirements();
    }

    @Override
    public void initialize() {
        if (SubsystemContainer.alliance.isRedAlliance()) {
            SubsystemContainer.limelightSubsystem.setPriorityID(4);
        } else {
            SubsystemContainer.limelightSubsystem.setPriorityID(7);
        }
        timer.start();
        SubsystemContainer.limelightSubsystem.forceMegaTagUpdate(true);
    }

    @Override
    public void execute() {
        SubsystemContainer.swerveSubsystem
                .autonRotateSwerve(SwerveSubsystem.calculateLimelightOffsetAngle());

        SubsystemContainer.swerveSubsystem.setCommandState(States.AUTOALIGN);

    }

    @Override
    public void end(boolean interrupted) {
        SubsystemContainer.limelightSubsystem.forceMegaTagUpdate(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}