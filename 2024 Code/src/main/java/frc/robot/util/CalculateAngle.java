package frc.robot.util;

import java.util.TreeMap;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.SubsystemContainer;

public class CalculateAngle {

    TreeMap<Double, Double> m_distanceAngleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    public CalculateAngle() {
        m_limelight = SubsystemContainer.limelightSubsystem;

        m_distanceAngleData.put(2.0, 40.0);
        m_distanceAngleData.put(4.0, 60.0);
        m_distanceAngleData.put(5.0, 70.0);
    }

    public double Interpolate() {
        double smallerDistance = 0;
        double largerDistance = 0;
        double resultAngle = 0;
        double distance = m_limelight.getDistance();

    }
}