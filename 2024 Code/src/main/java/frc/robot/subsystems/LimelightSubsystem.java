package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
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

  public boolean facingSpeakerID() {
    return getID() == RED_ALLIANCE_IDS.SPEAKER_ID || getID() == BLUE_ALLIANCE_IDS.SPEAKER_ID;
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

    /**
     * here are my thoughts lol and the basic overview of the structure that I have
     * in mind. Many of the numerical values
     * are subject to change as appropiate I just thought these ones made sense
     * based on past experience
     * 
     * key features
     * 
     * std needs to be a function of distance and take into account how many tags
     * are in view
     * the accepted "pose difference" also needs to be a function of distance, ie
     * the margin for accepted error is smaller when the robot is farther away
     * 
     * (these are rough numbers the functions that I have put in have been modified
     * slightly after looking at them in desmos)
     * domain of std function is 0 - 5
     * range of std function is 0 - 3
     * 
     * domain of accepted pose difference 0 - 5
     * range of accepted pose difference 0.75 - 0.2
     * 
     * first layer - reject all measurements that are outside the field
     * 1.5 layer - reject measurements when the robot is spinning
     * second layer - reject all measurements that are greater than 0.75m off the
     * current estimated position (maybe 0.5?) (this is done implicitly through the
     * function and max poseDifference accepted)
     * third layer - apply std function and accepted pose difference function. For
     * single tags, multiply the distance by 1.75, apply the functions as normal
     * fourth layer -
     * 
     * maybe more layers? This seemed sufficient to me but I could be forgetting
     * something
     */

    if (getTv() && !(Math.abs(SubsystemContainer.swerveSubsystem.getAngularVelocity()) > 720)) {
      limelight.getEntry("robot_orientation_set").setNumberArray(
          new Double[] { poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });

      // distance from current pose to vision estimated pose
      double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
          .getDistance(limelightPose.getTranslation());

      double distanceToSpeaker = SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getRadius();

      double distanceToPrimaryTag = pose[9];

      // increases the effective viewing distance for single tags because they are
      // less accurate.
      // This change means that the std will go up for a single tag and the
      // poseDifference threshold
      // will be even lower for a single tag at the same distance as a double
      if (pose[7] <= 1) {
        distanceToPrimaryTag *= 1.75;
      }

      // apply std linear function. As the distance increases, the applied standard
      // deviation goes up
      double xyStds = (3.0 / 5.0) * distanceToPrimaryTag + 0.1;

      // apply the accepted pose difference function. Returns the maximum pose
      // difference that is accepted
      // for a given distance input
      double maxPoseDifferenceAccepted = (-0.65 / 5.0) * distanceToPrimaryTag + 0.75;

      // the pose difference falls in the accepted range, as described by the linear
      // function above^^
      if (maxPoseDifferenceAccepted > poseDifference) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 0.0));

        poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (pose[6] / 1000.0));

        if (!(Math.abs(SubsystemContainer.swerveSubsystem.getAngularVelocity()) > 720)) {
          // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 0));
          // poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() -
          // (pose[6] / 1000.0));
        }
      }
    }
  }

  public void resetOdometryToLimelightPose() {
    if (getTv()) {
      SubsystemContainer.swerveSubsystem.ResetOdometryToPose(pose[0], pose[1]);
    }
  }
}