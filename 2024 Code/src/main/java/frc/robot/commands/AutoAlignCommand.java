// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlignCommand extends Command {
    double currentYaw;
    double targetYaw;
    LEDSubsystem m_LEDSubsystem;

    public AutoAlignCommand() {
        addRequirements(SubsystemContainer.limelightSubsystem);
    }

    @Override
    public void initialize() {
        SubsystemContainer.limelightSubsystem.setPipeline(1);
    }

    @Override
    public void execute() {
        currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        targetYaw = SubsystemContainer.limelightSubsystem.getTx();
        SubsystemContainer.swerveSubsystem.setRobotYaw(currentYaw - targetYaw);
        m_LEDSubsystem.isRunning = false;
        m_LEDSubsystem.doneRunning = true;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
