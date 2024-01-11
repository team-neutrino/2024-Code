// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Limiter;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.FRONT_RIGHT_COORD, SwerveConstants.FRONT_LEFT_COORD, 
  SwerveConstants.BACK_RIGHT_COORD, SwerveConstants.BACK_LEFT_COORD);
  AHRS m_navX = new AHRS();
  SwerveModuleState[] moduleStates;
  SwerveModule m_frontRight = new SwerveModule(SwerveConstants.FRS, SwerveConstants.FRA);
  SwerveModule m_frontLeft = new SwerveModule(SwerveConstants.FLS, SwerveConstants.FLA);
  SwerveModule m_backRight = new SwerveModule(SwerveConstants.BRS, SwerveConstants.BRA);
  SwerveModule m_backLeft = new SwerveModule(SwerveConstants.BLS, SwerveConstants.BLA);
  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(SwerveConstants.ks, SwerveConstants.kv);
  
  public SwerveSubsystem() {}

  public void Swerve(double vx, double vy, double omega){
    vx = Limiter.scale(Limiter.deadzone(vx, 0.2), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED, SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    vy = Limiter.scale(Limiter.deadzone(vy, 0.2), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED, SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    omega = Limiter.scale(Limiter.deadzone(omega, 0.2), -SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED, SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED);
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, omega);
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, Rotation2d.fromDegrees(getYaw()));
    moduleStates = m_kinematics.toSwerveModuleStates(robotSpeeds);
    moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], m_frontRight.getOptimizationAngle());
    moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], m_frontLeft.getOptimizationAngle());
    moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], m_backRight.getOptimizationAngle());
    moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], m_backLeft.getOptimizationAngle());
    double feedForwardFR = m_feedForward.calculate(moduleStates[0].speedMetersPerSecond);
    double feedForwardFL = m_feedForward.calculate(moduleStates[1].speedMetersPerSecond);
    double feedForwardBR = m_feedForward.calculate(moduleStates[2].speedMetersPerSecond);
    double feedForwardBL = m_feedForward.calculate(moduleStates[3].speedMetersPerSecond);
  }

  public double getYaw(){
    return m_navX.getYaw()*(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
