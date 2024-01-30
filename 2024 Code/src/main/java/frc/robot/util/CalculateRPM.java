package frc.robot.util;

import java.util.TreeMap;

import frc.robot.subsystems.ArmSubsystem;

public class CalculateRPM {

    TreeMap<Double, Double> m_angleRPMData = new TreeMap<Double, Double>();
    ArmSubsystem m_armSubsystem;

    public CalculateRPM() {
        m_armSubsystem = SubsystemContainer.armSubsystem;

        m_angleRPMData.put(25.0, 500.0);
        m_angleRPMData.put(50.0, 700.0);
        m_angleRPMData.put(75.0, 1000.0);
    }

    public double InterpolateRPM() {
        double smallerAngle = 0;
        double largerAngle = 0;
        double resultRPM = 0;
        double angle = m_armSubsystem.getTargetAngle();

        if (angle <= m_angleRPMData.firstKey()) {
            return m_angleRPMData.get(m_angleRPMData.firstKey());
        } else if (angle >= m_angleRPMData.lastKey()) {
            return m_angleRPMData.get(m_angleRPMData.lastKey());
        } else {
            for (Double a : m_angleRPMData.keySet()) {
                if (a >= angle) {
                    largerAngle = a;
                    break;
                } else {
                    smallerAngle = a;
                }
            }
        }
        resultRPM = m_angleRPMData.get(smallerAngle)
                + ((angle - smallerAngle))
                        * ((m_angleRPMData.get(largerAngle) - m_angleRPMData.get(smallerAngle))
                                / (largerAngle - smallerAngle));
        return resultRPM;
    }
}