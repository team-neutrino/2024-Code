package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
  private NetworkTableInstance instance;
  private NetworkTable photonVision;
  static PhotonTrackedTarget target;
  PhotonCamera camera;
  PhotonPipelineResult result;

  public PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult();
  }

  public boolean hasTarget() {
    return result.hasTargets();
  }

  public PhotonVision() {
    instance = NetworkTableInstance.getDefault();
    photonVision = instance.getTable("Microsoft_LifeCam_HD-3000");
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    result = camera.getLatestResult();
    // global instance of the network table and gets the limelight table
    // turns off LED
    photonVision.getEntry("ledMode").setNumber(1);
    camera.setPipelineIndex(2);
  }

  public Transform3d getInfo() {
    return target.getBestCameraToTarget();
  }

  public static double getYaw() {
    return target.getYaw();
  }

  @Override
  public void periodic() {
    // target = result.getBestTarget();
    // System.out.println(getYaw());
    // System.out.println(hasTarget());
    System.out.println("huboufbbfe");
  }

}
