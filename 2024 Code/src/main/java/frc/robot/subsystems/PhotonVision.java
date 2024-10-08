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
  static PhotonPipelineResult result;

  public PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult();
  }

  public boolean hasTarget() {
    return result.hasTargets();
  }

  public PhotonVision() {
    instance = NetworkTableInstance.getDefault();
    camera = new PhotonCamera("Arducam_B0478_(USB3_48MP)");
    photonVision = instance.getTable("/photonvision/Arducam_B0478_(USB3_48MP)");
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

  public PhotonPipelineResult getResult() {
    return result;
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    if (hasTarget()) {
      target = result.getBestTarget();
    }
  }

}
