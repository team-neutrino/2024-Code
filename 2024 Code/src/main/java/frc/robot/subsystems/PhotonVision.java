package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable photonVision = instance.getTable("Microsoft_LifeCam_HD-3000");
  static PhotonTrackedTarget target;
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  PhotonPipelineResult result = camera.getLatestResult();

  public PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult();
  }

  public boolean hasTarget() {
    return result.hasTargets();
  }

  public PhotonVision() {
    // global instance of the network table and gets the limelight table
    photonVision = NetworkTableInstance.getDefault().getTable("photonVision");
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

  public void periodic() {
    // target = result.getBestTarget();
    // System.out.println(getYaw());
    // System.out.println(hasTarget());
    for (int i = 0; i < 3; i++) {
      if (i % 3 == 0) {
        System.out.println("Working ....");
      } else {
        System.out.println("1.");
        System.out.println(result.hasTargets());
      }
    }
  }

}
