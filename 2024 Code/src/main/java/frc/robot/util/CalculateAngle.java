package frc.robot.util;

import java.util.TreeMap;

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
        m_angleData.put(-10.0, 3.0);
        m_angleData.put(-12.07, 5.8);
        m_angleData.put(-14.03, 6.5);
        m_angleData.put(-16.0, 9.5);
    }

    public double InterpolateAngle() {
        Double smallerAngle = 0.0;
        Double largerAngle = 0.0;
        double resultAngle = 0;
        double ty = m_limelight.getTy();

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

        return resultAngle;
    }
}