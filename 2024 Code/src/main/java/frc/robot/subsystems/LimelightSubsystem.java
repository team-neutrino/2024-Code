package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.simulation.LimelightSimulation;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable limelight;
  private double[] pose = new double[6];
  private double[] targetPose = new double[6];
  private double[] pastPose = new double[6];
  private double[] pastTargetPose = new double[6];

  public final LimelightSimulation m_LimelightSimulation = new LimelightSimulation();

  public LimelightSubsystem() {
    // global instance of the network table and gets the limelight table
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    // turns off LED
    limelight.getEntry("ledMode").setNumber(1);
  }

  public boolean getTv() {
    NetworkTableEntry tv = limelight.getEntry("tv");
    double validTarget = tv.getDouble(m_LimelightSimulation.GetTy());

    return validTarget == 1;
  }

  public Double getID() {
    Double id = limelight.getEntry("tid").getDouble(m_LimelightSimulation.GetId());
    return id;
  }

  // gets the x offest between the center of vision and the detected object
  public double getTx() {
    return limelight.getEntry("tx").getDouble(m_LimelightSimulation.GetTx());
  }

  // gets the y offest between the center of vision and the detected object
  public double getTy() {
    return limelight.getEntry("ty").getDouble(m_LimelightSimulation.GetTy());
  }

  public void periodic() {
    limelight.getEntry("ledMode").setNumber(1);
  }

  public double[] getBotPose() {
    pose = limelight.getEntry("botpose").getDoubleArray(pastPose);
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
}