package frc.robot.util;

import java.util.TreeMap;

import frc.robot.Constants.DimensionConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class CalculateAngle {

    TreeMap<Double, Double> m_angleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    public CalculateAngle() {
        m_limelight = SubsystemContainer.limelightSubsystem;

        m_angleData.put(11.97, -13.53);
        m_angleData.put(8.64, -13.48);
        m_angleData.put(3.5, -10.429);
        m_angleData.put(0.0, -4.029);
        m_angleData.put(-3.0, -4.0);
        m_angleData.put(-6.0, -1.7);
        m_angleData.put(-8.55, 2.308);
        m_angleData.put(-10.0, 2.837);
        m_angleData.put(-12.07, 5.5);
        m_angleData.put(-14.03, 5.85);
        m_angleData.put(-16.0, 8.46);
    }

    public double InterpolateAngle() {
        Double smallerAngle = 0.0;
        Double largerAngle = 0.0;
        double resultAngle = 0;
        double ty = m_limelight.getTy();

        // if (ty <= m_distanceAngleData.firstKey()) {
        // return m_distanceAngleData.get(m_distanceAngleData.firstKey());
        // } else if (ty >= m_distanceAngleData.lastKey()) {
        // return m_distanceAngleData.get(m_distanceAngleData.lastKey());
        // } else {
        // for (Double a : m_distanceAngleData.keySet()) {
        // if (a >= ty) {
        // largerDistance = a;
        // break;
        // } else {
        // smallerDistance = a;
        // }
        // }
        // }

        smallerAngle = m_angleData.lowerKey(ty);
        largerAngle = m_angleData.higherKey(ty);

        if (smallerAngle == null) {
            smallerAngle = m_angleData.firstKey();
        } else if (largerAngle == null) {
            largerAngle = m_angleData.lastKey();
        }

        resultAngle = (m_angleData.get(smallerAngle) * (largerAngle - ty)
                + m_angleData.get(largerAngle) * (ty - smallerAngle))
                / (largerAngle - smallerAngle);

        // resultAngle = m_angleData.get(smallerAngle)
        // + ((ty - smallerAngle))
        // * ((m_angleData.get(largerAngle) - m_angleData.get(smallerAngle))
        // / (largerAngle - smallerAngle));

        return resultAngle;
    }
}