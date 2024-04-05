package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable limelight;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];
  private double[] pastPose = new double[11];
  private double[] pastTargetPose = new double[6];

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

  public Double getID() {
    Double id = limelight.getEntry("tid").getDouble(0.0);
    return id;
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
    limelight.getEntry("ledMode").setNumber(1);
  }

  public double[] getBotPose() {
    pose = limelight.getEntry("botpose_wpiblue").getDoubleArray(pastPose);
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

    // distance from current pose to vision estimated pose
    double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(limelightPose.getTranslation());

    double distanceToSpeaker;

    if (SubsystemContainer.alliance.isRedAlliance()) {
      distanceToSpeaker = poseEstimator.getEstimatedPosition().getTranslation()
          .getDistance(SwerveConstants.SPEAKER_RED_SIDE);

    } else {
      distanceToSpeaker = poseEstimator.getEstimatedPosition().getTranslation()
          .getDistance(SwerveConstants.SPEAKER_BLUE_SIDE);
    }

    if (getTv()) {
      double xyStds = 1.0;

      // multiple targets detected
      if (pose[7] >= 2 && distanceToSpeaker < 2.8) {
        xyStds = 0.5;
      }
      // multiple targets but we don't want to trust it as much because the robot is
      // far away
      else if (pose[7] >= 2 && distanceToSpeaker < 4 && poseDifference < 0.2) {
        xyStds = 3.0;
      }
      // 1 target with large area and close to estimated pose (find constant for area
      // (percent))
      else if (pose[10] > 0.7 && poseDifference < 1) {
        xyStds = 1.0;
      }
      // 1 target farther away and estimated pose is close
      else if (pose[10] > 0.2 && poseDifference < 0.2) {
        xyStds = 3.0;
      }
      // conditions don't match to add a vision measurement
      else {
        return;
      }

      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 0.0));

      poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (pose[6] / 1000.0));

      limelight.getEntry("robot_orientation_set").setNumberArray(
          new Double[] { poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });
      if (Math.abs(SubsystemContainer.swerveSubsystem.getAngularVelocity()) > 720) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
        poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (pose[6] / 1000.0));
      }
    }
  }

  public void resetOdometryToLimelightPose() {
    if (getTv()) {
      SubsystemContainer.swerveSubsystem.ResetOdometryToPose(pose[0], pose[1]);
    }
  }
}