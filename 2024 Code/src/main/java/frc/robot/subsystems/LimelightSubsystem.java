package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants.BLUE_ALLIANCE_IDS;
import frc.robot.Constants.AprilTagConstants.RED_ALLIANCE_IDS;
import frc.robot.util.Alliance;
import frc.robot.util.SubsystemContainer;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable limelight;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];
  private double[] pastPose = new double[11];
  private double[] pastTargetPose = new double[6];
  private boolean m_forceUpdate = false;

  public LimelightSubsystem() {
    // global instance of the network table and gets the limelight table
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    // turns off LED
    limelight.getEntry("ledMode").setNumber(1);
  }

  public boolean getTv() {
    NetworkTableEntry tv = limelight.getEntry("tv");
    double validTarget = tv.getDouble(0.0);

    return validTarget == 1;
  }

  public double getDistanceFromPrimaryTarget() {
    return getBotPose()[9];
  }

  public int getID() {
    return (int) limelight.getEntry("tid").getDouble(0.0);
  }

  // gets the x offest between the center of vision and the detected object
  public double getTx() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  // gets the y offest between the center of vision and the detected object
  public double getTy() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public void periodic() {
    SwerveSubsystem swerve = SubsystemContainer.swerveSubsystem;
    limelight.getEntry("ledMode").setNumber(1);
    limelight.getEntry("robot_orientation_set").setNumberArray(
        new Double[] {
            swerve.m_swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
            0.0, 0.0, 0.0, 0.0, 0.0 });

    Pose2d botPose = new Pose2d();
    if (SubsystemContainer.alliance.isRedAlliance()) {
      botPose = new Pose2d(getBotPose()[0], getBotPose()[1],
          Rotation2d.fromDegrees(swerve.getYaw() + 180));
    } else {
      botPose = new Pose2d(getBotPose()[0], getBotPose()[1],
          Rotation2d.fromDegrees(swerve.getYaw()));
    }

    // ChassisSpeeds speeds = swerve.getRobotRelativeSpeeds();
    // if (Math.abs(speeds.vyMetersPerSecond) < 0.1 &&
    // Math.abs(speeds.vxMetersPerSecond) < 0.1) {
    if (!DriverStation.isAutonomousEnabled() || m_forceUpdate) {
      updatePoseEstimatorWithVisionBotPose(swerve.m_swervePoseEstimator, botPose);
    }
  }

  public double[] getBotPose() {
    // if (SubsystemContainer.alliance.isRedAlliance()) {
    // pose = limelight.getEntry("botpose_orb_wpired").getDoubleArray(pastPose);
    // } else {
    // pose = limelight.getEntry("botpose_orb_wpiblue").getDoubleArray(pastPose);
    // }
    pose = limelight.getEntry("botpose_orb_wpiblue").getDoubleArray(pastPose);
    if (getTv()) {
      pastPose = pose;
    }
    return pose;
  }

  public double[] getTargetPose() {
    targetPose = limelight.getEntry("targetpose_robotspace").getDoubleArray(pastTargetPose);
    if (getTv()) {
      pastTargetPose = targetPose;
    }
    return targetPose;
  }

  public void setPipeline(int pipeline) {
    limelight.getEntry("pipeline").setNumber(pipeline);
  }

  public void setPriorityID(int id) {
    limelight.getEntry("priorityid").setNumber(id);
  }

  public boolean facingSpeakerID() {
    return getID() == RED_ALLIANCE_IDS.SPEAKER_ID || getID() == BLUE_ALLIANCE_IDS.SPEAKER_ID;
  }

  public void forceMegaTagUpdate(boolean force) {
    m_forceUpdate = force;
  }

  /**
   * This method is primarily taken from the limelight docs page under "Robot
   * Localization with MegaTag." It specifies the conditions for accepting
   * a vision measurement and what the Stds should be depending on the
   * circumstance. This should hopefully improve the accuracy of the odometry and
   * reject
   * noisy/false data while keeping data that is correct.
   * 
   * @param poseEstimator
   * @param limelightPose
   */
  public void updatePoseEstimatorWithVisionBotPose(SwerveDrivePoseEstimator poseEstimator, Pose2d limelightPose) {
    // invalid limelight data
    if (limelightPose.getX() == 0.0) {
      return;
    }
    if (!(Math.abs(SubsystemContainer.swerveSubsystem.getAngularVelocity()) > 720) && getTv()) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 0));
      poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (pose[6] / 1000.0));
    }
  }

  public void resetOdometryToLimelightPose() {
    if (getTv()) {
      SubsystemContainer.swerveSubsystem.ResetOdometryToPose(pose[0], pose[1]);
    }
  }
}