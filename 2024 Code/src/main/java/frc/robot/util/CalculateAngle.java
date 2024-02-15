package frc.robot.util;

import java.util.TreeMap;

import frc.robot.Constants.DimensionConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class CalculateAngle {

    TreeMap<Double, Double> m_distanceAngleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    public CalculateAngle() {
        m_limelight = SubsystemContainer.limelightSubsystem;

        m_distanceAngleData.put(11.97, -13.53);
        m_distanceAngleData.put(8.64, -13.48);
        m_distanceAngleData.put(3.5, -10.429);
        m_distanceAngleData.put(0.0, -4.029);
        m_distanceAngleData.put(-3.0, -4.0);
        m_distanceAngleData.put(-6.0, -1.7);
        m_distanceAngleData.put(-8.55, 2.308);
        m_distanceAngleData.put(-10.0, 2.837);
        m_distanceAngleData.put(-12.07, 5.5);
        m_distanceAngleData.put(-14.03, 5.85);
        m_distanceAngleData.put(-16.0, 8.46);
    }

    public double InterpolateAngle() {
        double smallerDistance = 0;
        double largerDistance = 0;
        double resultAngle = 0;
        double ty = m_limelight.getTy();

        if (ty <= m_distanceAngleData.firstKey()) {
            return m_distanceAngleData.get(m_distanceAngleData.firstKey());
        } else if (ty >= m_distanceAngleData.lastKey()) {
            return m_distanceAngleData.get(m_distanceAngleData.lastKey());
        } else {
            for (Double a : m_distanceAngleData.keySet()) {
                if (a >= ty) {
                    largerDistance = a;
                    break;
                } else {
                    smallerDistance = a;
                }
            }
        }
        resultAngle = m_distanceAngleData.get(smallerDistance)
                + ((ty - smallerDistance))
                        * ((m_distanceAngleData.get(largerDistance) - m_distanceAngleData.get(smallerDistance))
                                / (largerDistance - smallerDistance));
        return resultAngle;
    }
}