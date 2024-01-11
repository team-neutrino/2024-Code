// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignCommand extends CommandBase {
  /** Creates a new AutoAlignCommand. */
  
  SwerveSubsystem m_swerveSubsystem;
  XboxController m_controller;
  LimelightSubsystem m_limelight;

  double LIMELIGHT_TO_METER_CONVERSION = 0.76189;
  double SWERVE_TIME = 5;

  double sideOffset;
  double frontOffset;
  double theta;
  Timer amogusTimer = new Timer();
  boolean boolsWorld;

  
  public AutoAlignCommand(SwerveSubsystem p_swerveSubsystem, LimelightSubsystem p_limelight, XboxController p_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = p_controller;
    m_swerveSubsystem = p_swerveSubsystem;
    m_limelight = p_limelight;
  
  }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(m_limelight.getTv() && !boolsWorld){
            double[] camTran = m_limelight.getCamTran();
            
            sideOffset = camTran[0] * LIMELIGHT_TO_METER_CONVERSION;
            frontOffset = camTran[2] * LIMELIGHT_TO_METER_CONVERSION;
            theta = Math.atan2(sideOffset, frontOffset);
            amogusTimer.start();
            boolsWorld = true;
            amogusTimer.reset();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double vx = sideOffset / SWERVE_TIME;
        double vy = frontOffset / SWERVE_TIME;
        double omega = theta / SWERVE_TIME;

        m_swerveSubsystem.Swerve(vx, vy, omega);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(amogusTimer.hasElapsed(SWERVE_TIME)){
            amogusTimer.stop();
            boolsWorld = false;
            return true;
        }
        return false;
    } 
}