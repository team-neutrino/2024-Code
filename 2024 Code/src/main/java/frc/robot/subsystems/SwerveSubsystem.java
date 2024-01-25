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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Limiter;

public class SwerveSubsystem extends SubsystemBase {
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.FRONT_RIGHT_COORD,
      SwerveConstants.FRONT_LEFT_COORD,
      SwerveConstants.BACK_RIGHT_COORD, SwerveConstants.BACK_LEFT_COORD);
  AHRS m_navX = new AHRS();
  SwerveModuleState[] moduleStates;

  private SwerveModule.MotorCfg front_right_speed = new SwerveModule.MotorCfg(MotorIDs.FRS,
      false);
  private final SwerveModule.MotorCfg front_left_speed = new SwerveModule.MotorCfg(MotorIDs.FLS,
      true);
  private final SwerveModule.MotorCfg back_right_speed = new SwerveModule.MotorCfg(MotorIDs.BRS,
      true);
  private final SwerveModule.MotorCfg back_left_speed = new SwerveModule.MotorCfg(MotorIDs.BLS,
      true);

  private final SwerveModule.MotorCfg front_right_angle = new SwerveModule.MotorCfg(MotorIDs.FRA,
      false, SwerveConstants.FRA_OFFSET);
  private final SwerveModule.MotorCfg front_left_angle = new SwerveModule.MotorCfg(MotorIDs.FLA,
      false, SwerveConstants.FLA_OFFSET);
  private final SwerveModule.MotorCfg back_right_angle = new SwerveModule.MotorCfg(MotorIDs.BRA,
      false, SwerveConstants.BRA_OFFSET);
  private final SwerveModule.MotorCfg back_left_angle = new SwerveModule.MotorCfg(MotorIDs.BLA,
      false, SwerveConstants.BLA_OFFSET);

  public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  private SwerveDriveOdometry m_swerveOdometry;

  private PIDController m_angleController = new PIDController(0.08, 0, 0);
  private Timer m_timer = new Timer();
  private double m_referenceAngle = 0;
  private boolean m_referenceSet = false;

  boolean omegaZero = false;

  SwerveModule m_frontRight = new SwerveModule(front_right_speed, front_right_angle);
  SwerveModule m_frontLeft = new SwerveModule(front_left_speed, front_left_angle);
  SwerveModule m_backRight = new SwerveModule(back_right_speed, back_right_angle);
  SwerveModule m_backLeft = new SwerveModule(back_left_speed, back_left_angle);
  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(SwerveConstants.ks, SwerveConstants.kv);

  double cycle = 0;

  Field2d field = new Field2d();
  Pose2d currentPose = new Pose2d();
  Pose2d generalSimPose = new Pose2d();
  public Command m_pathfindAmp;
  // public Command m_pathfind;

  public SwerveSubsystem() {
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    m_swerveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(getYaw()),
        modulePositions);

    m_angleController.enableContinuousInput(-180, 180);

    SmartDashboard.putData("Field", field);
    field.getRobotObject().close();

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::robotRelativeSwerve,
        new HolonomicPathFollowerConfig(
            new PIDConstants(0.4, 0.0, 0.0),
            new PIDConstants(1.4, 0.0, 0.0),
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

    if (isRedAlliance() == true) {
      m_pathfindAmp = AutoBuilder.pathfindToPose(new Pose2d(SwerveConstants.AMP_TARGET_POSE_RED, new Rotation2d()),
          Constants.SwerveConstants.PATH_CONSTRAINTS);
    } else {
      m_pathfindAmp = AutoBuilder.pathfindToPose(new Pose2d(SwerveConstants.AMP_TARGET_POSE_BLUE, new Rotation2d()),
          Constants.SwerveConstants.PATH_CONSTRAINTS);
    }
  }

  public void Swerve(double vx, double vy, double omega) {
    vx = Limiter.scale(Limiter.deadzone(vx, 0.2), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED,
        SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    vy = Limiter.scale(Limiter.deadzone(vy, 0.2), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED,
        SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    omega = Limiter.scale(Limiter.deadzone(omega, 0.2), -SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED,
        SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED);

    if (omega == 0) {
      omegaZero = true;
    } else {
      omegaZero = false;
    }

    if (omega == 0 && m_timer.get() == 0) {
      m_timer.start();
    } else if (m_timer.get() >= 0.2 && !m_referenceSet) {
      m_referenceAngle = getYaw();
      m_referenceSet = true;
      m_timer.stop();
      m_timer.reset();
    } else if (omega != 0) {
      m_referenceSet = false;

    } else if (omega == 0 && m_referenceSet) {
      omega += m_angleController.calculate(getYaw(), m_referenceAngle);
    }

    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, omega);
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, Rotation2d.fromDegrees(getYaw()));

    moduleStates = m_kinematics.toSwerveModuleStates(robotSpeeds);

    moduleStates[0] = SwerveModuleState.optimize(moduleStates[0],
        m_frontRight.getOptimizationAngle());
    moduleStates[1] = SwerveModuleState.optimize(moduleStates[1],
        m_frontLeft.getOptimizationAngle());
    moduleStates[2] = SwerveModuleState.optimize(moduleStates[2],
        m_backRight.getOptimizationAngle());
    moduleStates[3] = SwerveModuleState.optimize(moduleStates[3],
        m_backLeft.getOptimizationAngle());

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

  public boolean omegaZero() {
    return omegaZero;
  }

  public void robotRelativeSwerve(ChassisSpeeds referenceSpeeds) {
    moduleStates = m_kinematics.toSwerveModuleStates(referenceSpeeds);

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

  public void setRobotYaw(double angle) {
    m_referenceAngle = angle;
  }

  public double getYaw() {
    return m_navX.getYaw() * (-1);
  }

  public void resetNavX() {
    m_navX.reset();
    m_referenceAngle = 0;
    m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), modulePositions,
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void resetStartPosition(Translation2d startPosition) {
    m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), modulePositions,
        new Pose2d(startPosition, Rotation2d.fromDegrees(getYaw())));
  }

  public Pose2d getPose() {
    return m_swerveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), modulePositions, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontRight.getModuleState(),
        m_frontLeft.getModuleState(),
        m_backRight.getModuleState(), m_backLeft.getModuleState());
  }

  /**
   * Returns true if the current alliance is red, false otherwise.
   * 
   * @return boolean representing the current alliance as retrieved from the
   *         Driver Station.
   */
  public boolean isRedAlliance() {
    boolean isRed = false;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRed = alliance.get() == DriverStation.Alliance.Red;
    }

    return isRed;
  }

  public Command getPathfindCommand() {
    boolean isRed = isRedAlliance();

    Pose2d closestPose;
    if (isRed) {
      closestPose = SwerveConstants.RED_TARGET_POSE1;

      double d2 = distanceFormula(SwerveConstants.RED_TARGET_POSE2);
      double d3 = distanceFormula(SwerveConstants.RED_TARGET_POSE3);
      double d4 = distanceFormula(SwerveConstants.RED_TARGET_POSE4);

      if (d2 < distanceFormula(closestPose)) {
        closestPose = SwerveConstants.RED_TARGET_POSE2;
      }
      if (d3 < distanceFormula(closestPose)) {
        closestPose = SwerveConstants.RED_TARGET_POSE3;
      }
      if (d4 < distanceFormula(closestPose)) {
        closestPose = SwerveConstants.RED_TARGET_POSE4;
      }
    } else {
      closestPose = SwerveConstants.BLUE_TARGET_POSE1;

      double d2 = distanceFormula(SwerveConstants.BLUE_TARGET_POSE2);
      double d3 = distanceFormula(SwerveConstants.BLUE_TARGET_POSE3);
      double d4 = distanceFormula(SwerveConstants.BLUE_TARGET_POSE4);

      if (d2 < distanceFormula(closestPose)) {
        closestPose = SwerveConstants.BLUE_TARGET_POSE2;
      }
      if (d3 < distanceFormula(closestPose)) {
        closestPose = SwerveConstants.BLUE_TARGET_POSE3;
      }
      if (d4 < distanceFormula(closestPose)) {
        closestPose = SwerveConstants.BLUE_TARGET_POSE4;
      }

    }

    return AutoBuilder.pathfindToPose(closestPose, SwerveConstants.PATH_CONSTRAINTS);
  }

  public double distanceFormula(Pose2d targetPose) {
    Pose2d current = currentPose;
    return Math.sqrt(Math.pow(targetPose.getX() - current.getX(), 2)
        + Math.pow(targetPose.getY() - current.getY(), 2));
  }

  @Override
  public void periodic() {
    modulePositions[0] = m_frontRight.getModulePosition();
    modulePositions[1] = m_frontLeft.getModulePosition();
    modulePositions[2] = m_backRight.getModulePosition();
    modulePositions[3] = m_backLeft.getModulePosition();

    currentPose = m_swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), modulePositions);

    field.getObject("auton").setPose(currentPose);

    cycle++;
    if (cycle % 8 == 0) {

    }
  }
}