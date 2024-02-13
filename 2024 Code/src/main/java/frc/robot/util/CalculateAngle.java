package frc.robot.util;

import java.util.TreeMap;

import frc.robot.Constants.DimensionConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class CalculateAngle {

    TreeMap<Double, Double> m_distanceAngleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    public CalculateAngle() {
        m_limelight = SubsystemContainer.limelightSubsystem;

        m_distanceAngleData.put(-10.0, 40.0);
        m_distanceAngleData.put(4.0, 60.0);
        m_distanceAngleData.put(10.0, 70.0);
    }

    public double InterpolateAngle() {
        double smallerDistance = 0;
        double largerDistance = 0;
        double resultAngle = 0;
        double distance = DimensionConstants.SPEAKER_TO_MOUNT_HEIGHT / Math.tan(Math.toRadians(m_limelight.getTy()));

        if (distance <= m_distanceAngleData.firstKey()) {
            return m_distanceAngleData.get(m_distanceAngleData.firstKey());
        } else if (distance >= m_distanceAngleData.lastKey()) {
            return m_distanceAngleData.get(m_distanceAngleData.lastKey());
        } else {
            for (Double a : m_distanceAngleData.keySet()) {
                if (a >= distance) {
                    largerDistance = a;
                    break;
                } else {
                    smallerDistance = a;
                }
            }
        }
        resultAngle = m_distanceAngleData.get(smallerDistance)
                + ((distance - smallerDistance))
                        * ((m_distanceAngleData.get(largerDistance) - m_distanceAngleData.get(smallerDistance))
                                / (largerDistance - smallerDistance));
        return resultAngle;
    }
}