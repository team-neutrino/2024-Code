// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
    double[] pose;
    double yawChange;
    private LimelightSubsystem m_limelight;
    private SwerveSubsystem m_swerveSubsystem;

    public AutoAlign() {
        m_limelight = SubsystemContainer.limelightSubsystem;
        m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
        addRequirements(m_limelight, m_swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // in degrees?
        if(m_limelight.getID().equals(4.0)){
            
        } else if(m_limelight.getID().equals(7.0)){

        }
        m_swerveSubsystem.setRobotYaw();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
