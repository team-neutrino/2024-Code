// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Limiter;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.FRONT_RIGHT_COORD,
      SwerveConstants.FRONT_LEFT_COORD,
      SwerveConstants.BACK_RIGHT_COORD, SwerveConstants.BACK_LEFT_COORD);
  AHRS m_navX = new AHRS();
  SwerveModuleState[] moduleStates;

  private SwerveModule.MotorCfg front_right_speed = new SwerveModule.MotorCfg(MotorIDs.FRS,
      true);
  private final SwerveModule.MotorCfg front_left_speed = new SwerveModule.MotorCfg(MotorIDs.FLS,
      false);
  private final SwerveModule.MotorCfg back_right_speed = new SwerveModule.MotorCfg(MotorIDs.BRS,
      false);
  private final SwerveModule.MotorCfg back_left_speed = new SwerveModule.MotorCfg(MotorIDs.BLS,
      false);

  private final SwerveModule.MotorCfg front_right_angle = new SwerveModule.MotorCfg(MotorIDs.FRA,
      false, SwerveConstants.FRA_OFFSET);
  private final SwerveModule.MotorCfg front_left_angle = new SwerveModule.MotorCfg(MotorIDs.FLA,
      false, SwerveConstants.FLA_OFFSET);
  private final SwerveModule.MotorCfg back_right_angle = new SwerveModule.MotorCfg(MotorIDs.BRA,
      false, SwerveConstants.BRA_OFFSET);
  private final SwerveModule.MotorCfg back_left_angle = new SwerveModule.MotorCfg(MotorIDs.BLA,
      false, SwerveConstants.BLA_OFFSET);

  public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  private PIDController m_angleController = new PIDController(1, 0, 0);

  private SwerveDriveOdometry m_swerveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(getYaw()),
      modulePositions);

  SwerveModule m_frontRight = new SwerveModule(front_right_speed, front_right_angle);
  SwerveModule m_frontLeft = new SwerveModule(front_left_speed, front_left_angle);
  SwerveModule m_backRight = new SwerveModule(back_right_speed, back_right_angle);
  SwerveModule m_backLeft = new SwerveModule(back_left_speed, back_left_angle);
  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(SwerveConstants.ks, SwerveConstants.kv);

  double cycle = 0;

  public SwerveSubsystem() {
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    m_angleController.enableContinuousInput(-180, 180);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::robotRelativeSwerve,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1, 0.0, 0.0),
            new PIDConstants(1, 0.0, 0.0),
            SwerveConstants.MAX_MODULE_LINEAR_SPEED,
            SwerveConstants.DRIVEBASE_RADIUS,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public void Swerve(double vx, double vy, double omega) {
    vx = Limiter.scale(Limiter.deadzone(vx, 0.2), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED,
        SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    vy = Limiter.scale(Limiter.deadzone(vy, 0.2), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED,
        SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    omega = Limiter.scale(Limiter.deadzone(omega, 0.2), -SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED,
        SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED);

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

    for (int i = 0; i < 4; i++) {
      if (moduleStates[i].angle.getDegrees() <= 0) {
        moduleStates[i].angle = Rotation2d.fromDegrees(moduleStates[i].angle.getDegrees() * -1);
      } else {
        moduleStates[i].angle = Rotation2d.fromDegrees(360 - moduleStates[i].angle.getDegrees());
      }
    }

    m_frontRight.setAnglePID(moduleStates[0].angle.getDegrees());
    m_frontLeft.setAnglePID(moduleStates[1].angle.getDegrees());
    m_backRight.setAnglePID(moduleStates[2].angle.getDegrees());
    m_backLeft.setAnglePID(moduleStates[3].angle.getDegrees());

    m_frontRight.setSpeedPID(moduleStates[0].speedMetersPerSecond, feedForwardFR);
    m_frontLeft.setSpeedPID(moduleStates[1].speedMetersPerSecond, feedForwardFL);
    m_backRight.setSpeedPID(moduleStates[2].speedMetersPerSecond, feedForwardBR);
    m_backLeft.setSpeedPID(moduleStates[3].speedMetersPerSecond, feedForwardBL);
  }

  public void robotRelativeSwerve(ChassisSpeeds referenceSpeeds) {
    moduleStates = m_kinematics.toSwerveModuleStates(referenceSpeeds);

    moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], m_frontRight.getOptimizationAngle());
    moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], m_frontLeft.getOptimizationAngle());
    moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], m_backRight.getOptimizationAngle());
    moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], m_backLeft.getOptimizationAngle());

    double feedForwardFR = m_feedForward.calculate(moduleStates[0].speedMetersPerSecond);
    double feedForwardFL = m_feedForward.calculate(moduleStates[1].speedMetersPerSecond);
    double feedForwardBR = m_feedForward.calculate(moduleStates[2].speedMetersPerSecond);
    double feedForwardBL = m_feedForward.calculate(moduleStates[3].speedMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      if (moduleStates[i].angle.getDegrees() <= 0) {
        moduleStates[i].angle = Rotation2d.fromDegrees(moduleStates[i].angle.getDegrees() * -1);
      } else {
        moduleStates[i].angle = Rotation2d.fromDegrees(360 - moduleStates[i].angle.getDegrees());
      }
    }

    m_frontRight.setAnglePID(moduleStates[0].angle.getDegrees());
    m_frontLeft.setAnglePID(moduleStates[1].angle.getDegrees());
    m_backRight.setAnglePID(moduleStates[2].angle.getDegrees());
    m_backLeft.setAnglePID(moduleStates[3].angle.getDegrees());

    m_frontRight.setSpeedPID(moduleStates[0].speedMetersPerSecond, feedForwardFR);
    m_frontLeft.setSpeedPID(moduleStates[1].speedMetersPerSecond, feedForwardFL);
    m_backRight.setSpeedPID(moduleStates[2].speedMetersPerSecond, feedForwardBR);
    m_backLeft.setSpeedPID(moduleStates[3].speedMetersPerSecond, feedForwardBL);
  }

  public double getYaw() {
    return m_navX.getYaw() * (-1);
  }

  public void resetNavX() {
    m_navX.reset();
  }

  public Pose2d getPose() {
    return m_swerveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(m_navX.getAngle()), modulePositions, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontRight.getModuleState(), m_frontLeft.getModuleState(),
        m_backRight.getModuleState(), m_backLeft.getModuleState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    modulePositions[0] = m_frontRight.getModulePosition();
    modulePositions[1] = m_frontLeft.getModulePosition();
    modulePositions[2] = m_backRight.getModulePosition();
    modulePositions[3] = m_backLeft.getModulePosition();

    cycle++;
    if (cycle % 8 == 0) {

    }
  }
}
