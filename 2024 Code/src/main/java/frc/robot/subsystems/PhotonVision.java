package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private NetworkTable photonVision;
  private double[] pose = new double[6];
  private double[] targetPose = new double[6];
  private double[] pastPose = new double[6];
  private double[] pastTargetPose = new double[6];
  static PhotonTrackedTarget target;
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  public PhotonVision() {
    // global instance of the network table and gets the limelight table
    photonVision = NetworkTableInstance.getDefault().getTable("limelight");
    // turns off LED
    photonVision.getEntry("ledMode").setNumber(1);
  }

  public Transform3d getInfo() {
    return target.getBestCameraToTarget();
  }

  public static double getYaw() {
    return target.getYaw();
  }

  public void periodic() {
    var result = camera.getLatestResult();
    target = result.getBestTarget();
  }
}
