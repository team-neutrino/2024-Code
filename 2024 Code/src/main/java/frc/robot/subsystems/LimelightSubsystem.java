package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants.BLUE_ALLIANCE_IDS;
import frc.robot.Constants.AprilTagConstants.RED_ALLIANCE_IDS;
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

  /**
   * gets the x offest between the center of vision and the detected object
   */
  public double getTx() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  // gets the y offest between the center of vision and the detected object
  public double getTy() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public void periodic() {
    CommandSwerveDrivetrain swerve = SubsystemContainer.swerveSubsystem2;
    limelight.getEntry("ledMode").setNumber(1);
    limelight.getEntry("robot_orientation_set").setNumberArray(
        new Double[] {
            swerve.getSwervePoseEstimator().getEstimatedPosition().getRotation().getDegrees(),
            0.0, 0.0, 0.0, 0.0, 0.0 });

    double yaw = swerve.getYaw2() + (SubsystemContainer.alliance.isRedAlliance() ? 180 : 0);
    Pose2d botPose = new Pose2d(getBotPose()[0], getBotPose()[1], Rotation2d.fromDegrees(yaw));

    if (!DriverStation.isAutonomousEnabled() || m_forceUpdate) {
      updatePoseEstimatorWithVisionBotPose(swerve.getSwervePoseEstimator(), botPose);
    }
  }

  public double[] getBotPose() {
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
    // if (limelightPose.getX() == 0.0) {
    // return;
    // }
    // if (!(Math.abs(SubsystemContainer.swerveSubsystem.getAngularVelocity()) >
    // 720) && getTv()) {
    // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 0));
    // poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() -
    // (pose[6] / 1000.0));
    // }
  }

  public void resetOdometryToLimelightPose() {
    if (getTv()) {
      SubsystemContainer.swerveSubsystem.ResetOdometryToPose(pose[0], pose[1]);
    }
  }
}