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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTablesJNI;
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
import frc.robot.Constants.AprilTagConstants.BLUE_ALLIANCE_IDS;
import frc.robot.Constants.AprilTagConstants.RED_ALLIANCE_IDS;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.util.Limiter;
import frc.robot.util.PolarCoord;
import frc.robot.util.SubsystemContainer;

public class SwerveSubsystem extends SubsystemBase {
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.FRONT_RIGHT_COORD,
      SwerveConstants.FRONT_LEFT_COORD,
      SwerveConstants.BACK_RIGHT_COORD, SwerveConstants.BACK_LEFT_COORD);
  private AHRS m_navX = new AHRS();
  private SwerveModuleState[] m_moduleStates;

  private final SwerveModule.MotorCfg m_frontRightSpeed = new SwerveModule.MotorCfg(MotorIDs.FRS,
      true);
  private final SwerveModule.MotorCfg m_frontLeftSpeed = new SwerveModule.MotorCfg(MotorIDs.FLS,
      true);
  private final SwerveModule.MotorCfg m_backRightSpeed = new SwerveModule.MotorCfg(MotorIDs.BRS,
      true);
  private final SwerveModule.MotorCfg m_backLeftSpeed = new SwerveModule.MotorCfg(MotorIDs.BLS,
      false);

  private final SwerveModule.MotorCfg m_frontRightAngle = new SwerveModule.MotorCfg(MotorIDs.FRA,
      false, SwerveConstants.FRA_OFFSET);
  private final SwerveModule.MotorCfg m_frontLeftAngle = new SwerveModule.MotorCfg(MotorIDs.FLA,
      false, SwerveConstants.FLA_OFFSET);
  private final SwerveModule.MotorCfg m_backRightAngle = new SwerveModule.MotorCfg(MotorIDs.BRA,
      false, SwerveConstants.BRA_OFFSET);
  private final SwerveModule.MotorCfg m_backLeftAngle = new SwerveModule.MotorCfg(MotorIDs.BLA,
      false, SwerveConstants.BLA_OFFSET);

  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];

  private SwerveDriveOdometry m_swerveOdometry;

  public SwerveDrivePoseEstimator m_swervePoseEstimator;

  private PIDController m_angleController = new PIDController(0.06, 0, 0);
  private Timer m_timer = new Timer();
  private double m_referenceAngle = 0;
  private boolean m_referenceSet = false;

  private SwerveModule m_frontRight = new SwerveModule(m_frontRightSpeed, m_frontRightAngle);
  private SwerveModule m_frontLeft = new SwerveModule(m_frontLeftSpeed, m_frontLeftAngle);
  private SwerveModule m_backRight = new SwerveModule(m_backRightSpeed, m_backRightAngle);
  private SwerveModule m_backLeft = new SwerveModule(m_backLeftSpeed, m_backLeftAngle);

  private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(SwerveConstants.ks, SwerveConstants.kv);

  private Field2d m_field = new Field2d();
  private Pose2d m_currentPose = new Pose2d();
  private Pose2d m_currentPoseL = new Pose2d();
  private PolarCoord m_speakerToRobot = new PolarCoord();
  private Command m_pathfindAmp;

  private SlewRateLimiter m_filterX = new SlewRateLimiter(2);
  private SlewRateLimiter m_filterY = new SlewRateLimiter(2);
  private SlewRateLimiter m_filterOmega = new SlewRateLimiter(10.0);

  private States m_state;

  /**
   * Internal variables for updating the robot's current (non-rotational)
   * velocity.
   */
  private Pose2d lastPose = m_currentPoseL;
  private double lastTime = NetworkTablesJNI.now();;

  public SwerveSubsystem() {
    m_modulePositions[0] = new SwerveModulePosition();
    m_modulePositions[1] = new SwerveModulePosition();
    m_modulePositions[2] = new SwerveModulePosition();
    m_modulePositions[3] = new SwerveModulePosition();

    m_swerveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(getYaw()),
        m_modulePositions);

    m_swervePoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(getYaw()),
        m_modulePositions, new Pose2d());

    m_angleController.enableContinuousInput(-180, 180);

    SmartDashboard.putData("Field", m_field);
    m_field.getRobotObject().close();

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::robotRelativeSwerve,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0.0, 0.0),
            new PIDConstants(1.0, 0.0, 0.0), // 0.6 before
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

    if (SubsystemContainer.alliance.isRedAlliance()) {
      m_pathfindAmp = AutoBuilder.pathfindToPose(new Pose2d(SwerveConstants.AMP_TARGET_POSE_RED, new Rotation2d(-90)),
          Constants.SwerveConstants.PATH_CONSTRAINTS);
    } else {
      m_pathfindAmp = AutoBuilder.pathfindToPose(new Pose2d(SwerveConstants.AMP_TARGET_POSE_BLUE, new Rotation2d(-90)),
          Constants.SwerveConstants.PATH_CONSTRAINTS);
    }
  }

  /**
   * Returns the speed of the drivetrain in m/s. This method has not been tested
   * for accuracy with friction.
   * 
   * @return The current speed of the drivetrain in m/s.
   */
  public double getDriveMotorSpeed() {
    double currentTime = NetworkTablesJNI.now();
    double currentX = m_currentPoseL.getX();
    double currentY = m_currentPoseL.getY();

    double timeChange = currentTime - lastTime;
    double xChange = currentX - lastPose.getX();
    double yChange = currentY - lastPose.getY();

    lastTime = currentTime;
    lastPose = new Pose2d(currentX, currentY, m_currentPoseL.getRotation());

    return Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2)) / timeChange;
  }

  public void SwerveWithDeadzone(double vx, double vy, double omega) {
    vx = Limiter.deadzone(vx, 0.1);
    vy = Limiter.deadzone(vy, 0.1);
    omega = Limiter.deadzone(omega, 0.1);
    SwerveWithoutDeadzone(vx, vy, omega);
  }

  public void SwerveWithoutDeadzone(double vx, double vy, double omega) {
    if (vx != 0 && vy != 0) {
      vx = m_filterX.calculate(vx);
      vy = m_filterY.calculate(vy);
      omega = m_filterOmega.calculate(omega);
    }

    vx = Limiter.scale(Limiter.deadzone(vx, 0.1), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED,
        SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    vy = Limiter.scale(Limiter.deadzone(vy, 0.1), -SwerveConstants.MAX_CHASSIS_LINEAR_SPEED,
        SwerveConstants.MAX_CHASSIS_LINEAR_SPEED);
    omega = Limiter.scale(Limiter.deadzone(omega, 0.1), -SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED,
        SwerveConstants.MAX_CHASSIS_ROTATIONAL_SPEED);

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

    robotRelativeSwerve(robotSpeeds);
  }

  public void autonRotateSwerve(double reference) {
    double omega = m_angleController.calculate(getYaw(), reference);

    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(0, 0, omega);
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, Rotation2d.fromDegrees(getYaw()));

    robotRelativeSwerve(robotSpeeds);
  }

  private void robotRelativeSwerve(ChassisSpeeds referenceSpeeds) {
    m_moduleStates = m_kinematics.toSwerveModuleStates(referenceSpeeds);

    m_moduleStates[0] = SwerveModuleState.optimize(m_moduleStates[0],
        m_frontRight.getOptimizationAngle());
    m_moduleStates[1] = SwerveModuleState.optimize(m_moduleStates[1],
        m_frontLeft.getOptimizationAngle());
    m_moduleStates[2] = SwerveModuleState.optimize(m_moduleStates[2],
        m_backRight.getOptimizationAngle());
    m_moduleStates[3] = SwerveModuleState.optimize(m_moduleStates[3],
        m_backLeft.getOptimizationAngle());

    double feedForwardFR = m_feedForward.calculate(m_moduleStates[0].speedMetersPerSecond);
    double feedForwardFL = m_feedForward.calculate(m_moduleStates[1].speedMetersPerSecond);
    double feedForwardBR = m_feedForward.calculate(m_moduleStates[2].speedMetersPerSecond);
    double feedForwardBL = m_feedForward.calculate(m_moduleStates[3].speedMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      if (m_moduleStates[i].angle.getDegrees() <= 0) {
        m_moduleStates[i].angle = Rotation2d.fromDegrees(m_moduleStates[i].angle.getDegrees() * -1);
      } else {
        m_moduleStates[i].angle = Rotation2d.fromDegrees(360 - m_moduleStates[i].angle.getDegrees());
      }
    }

    m_frontRight.setAnglePID(m_moduleStates[0].angle.getDegrees());
    m_frontLeft.setAnglePID(m_moduleStates[1].angle.getDegrees());
    m_backRight.setAnglePID(m_moduleStates[2].angle.getDegrees());
    m_backLeft.setAnglePID(m_moduleStates[3].angle.getDegrees());

    m_frontRight.setSpeedPID(m_moduleStates[0].speedMetersPerSecond, feedForwardFR);
    m_frontLeft.setSpeedPID(m_moduleStates[1].speedMetersPerSecond, feedForwardFL);
    m_backRight.setSpeedPID(m_moduleStates[2].speedMetersPerSecond, feedForwardBR);
    m_backLeft.setSpeedPID(m_moduleStates[3].speedMetersPerSecond, feedForwardBL);
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

    if (SubsystemContainer.alliance.isRedAlliance()) {
      m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()),
          m_modulePositions,
          new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
      m_swervePoseEstimator.resetPosition(Rotation2d.fromDegrees(getYaw()),
          m_modulePositions,
          new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
    } else {
      m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()),
          m_modulePositions, new Pose2d());
      m_swervePoseEstimator.resetPosition(Rotation2d.fromDegrees(getYaw()),
          m_modulePositions, new Pose2d());
    }
    m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), m_modulePositions, new Pose2d());
    m_swervePoseEstimator.resetPosition(Rotation2d.fromDegrees(getYaw()), m_modulePositions, new Pose2d());
  }

  public void ResetModules() {
    m_frontRight.resetEverything();
    m_frontLeft.resetEverything();
    m_backRight.resetEverything();
    m_backLeft.resetEverything();
  }

  public Pose2d getPose() {
    return m_swervePoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), m_modulePositions, pose);
    m_swervePoseEstimator.resetPosition(Rotation2d.fromDegrees(getYaw()), m_modulePositions, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontRight.getModuleState(),
        m_frontLeft.getModuleState(),
        m_backRight.getModuleState(), m_backLeft.getModuleState());
  }

  /**
   * D-pad addition: pressing any of the 4 main buttons on the D-pad
   * serve as hotkeys for rotation to forward, backward, left, and right
   * relative to m_field orientation.
   * 
   * @param pov The current angle of the xbox controller POV buttons, -1
   *            if not pressed and otherwise increases clockwise from 0-359.
   */
  public void POV(double pov) {

    int integerPOV = Math.round((float) pov);

    if (integerPOV == 270) {
      // needed because of weird robot orientation (0 to -180 from left and 0 to +180
      // from right)
      setRobotYaw(90);
    } else if (integerPOV == 90) {
      setRobotYaw(-90);
    } else if (integerPOV >= 0) {
      setRobotYaw(integerPOV);
    }
  }

  private PolarCoord UpdateSpeakerToRobot(Pose2d pose) {
    double xComp = 0;
    double yComp = 0;
    double radius = 0.0;
    double theta = 0.0;
    if (SubsystemContainer.alliance.isRedAlliance()) {
      xComp = Math.abs(pose.getX() - SwerveConstants.SPEAKER_RED_SIDE.getX());
      yComp = Math.abs(pose.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY());
      radius = Math.sqrt(Math.pow(xComp, 2) + Math.pow(yComp, 2));
      theta = Math.atan(yComp / xComp);
    } else {
      xComp = Math.abs(pose.getX());
      yComp = Math.abs(pose.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY());
      radius = Math.sqrt(Math.pow(xComp, 2) + Math.pow(yComp, 2));
      theta = Math.atan(yComp / xComp);
    }

    return new PolarCoord(radius, theta);
  }

  /**
   * Returns a PolarCoord intended to be used in the ShootWhileSwerving command
   * that is a horizontally (relative to the speaker) translated version of
   * what m_speakerToRobot would normally be.
   * 
   * @param pose The robot's current position.
   * @return A PolarCoord consisting of the angle to the speaker and a translated
   *         distance determined by the difference between the given pose2d and
   *         the robot's current pose2d
   */
  public PolarCoord getShiftedSpeakerToRobot(Pose2d pose) {
    return UpdateSpeakerToRobot(pose);
  }

  public void AlignToSpeakerUsingOdometry() {
    Translation2d speakerPose;
    if (SubsystemContainer.alliance.isRedAlliance()) {
      SubsystemContainer.limelightSubsystem.setPriorityID(RED_ALLIANCE_IDS.SPEAKER_ID);
      speakerPose = SwerveConstants.SPEAKER_RED_SIDE;
    } else {
      SubsystemContainer.limelightSubsystem.setPriorityID(BLUE_ALLIANCE_IDS.SPEAKER_ID);
      speakerPose = SwerveConstants.SPEAKER_BLUE_SIDE;
    }

    setRobotYaw(Math.toDegrees(
        Math.atan((m_currentPoseL.getY() - speakerPose.getY()) / (m_currentPoseL.getX() - speakerPose.getX()))));
  }

  /**
   * Get x distance from amp, used in amp auto align
   * 
   * @return The robot's current x distance from the amp
   */
  public double getAmpDx() {
    if (SubsystemContainer.alliance.isRedAlliance()) {
      return m_currentPoseL.getX() - SwerveConstants.AMP_TARGET_POSE_RED.getX();
    } else {
      return m_currentPoseL.getX() - SwerveConstants.AMP_TARGET_POSE_BLUE.getX();
    }
  }

  public void ResetOdometryToPose(double x, double y) {
    resetPose(new Pose2d(x, y, m_currentPoseL.getRotation()));
  }

  public PolarCoord GetSpeakerToRobot() {
    return m_speakerToRobot;
  }

  public static double calculateLimelightOffsetAngle(double currentYaw, double offsetYaw, double robotTheta) {
    return currentYaw - offsetYaw + (robotTheta * 0.06);
  }

  public States getCommandState() {
    return m_state;
  }

  public void setCommandState(States p_state) {
    m_state = p_state;
  }

  @Override
  public void periodic() {
    System.out.println("Drivetrain speed: " + getDriveMotorSpeed());

    m_modulePositions[0] = m_frontRight.getModulePosition();
    m_modulePositions[1] = m_frontLeft.getModulePosition();
    m_modulePositions[2] = m_backRight.getModulePosition();
    m_modulePositions[3] = m_backLeft.getModulePosition();

    m_currentPose = m_swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), m_modulePositions);

    m_currentPoseL = m_swervePoseEstimator.update(Rotation2d.fromDegrees(getYaw()), m_modulePositions);
    m_speakerToRobot = UpdateSpeakerToRobot(m_currentPoseL);

    m_field.getObject("odometry w/o limelight").setPose(m_currentPose);
    m_field.getObject("with limelight").setPose(m_currentPoseL);
  }
}