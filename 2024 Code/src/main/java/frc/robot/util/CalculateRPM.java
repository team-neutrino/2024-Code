package frc.robot.util;

import java.util.TreeMap;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class CalculateRPM {

    TreeMap<Double, Double> m_angleRPMData = new TreeMap<Double, Double>();
    ArmSubsystem m_armSubsystem;

    public CalculateRPM() {
        m_armSubsystem = SubsystemContainer.armSubsystem;

        m_angleRPMData.put(-10.0, 40.0);
        m_angleRPMData.put(4.0, 60.0);
        m_angleRPMData.put(10.0, 70.0);
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
                        * ((m_angleRPMData.get(largerAngle) - m_angleRPMData.get(largerAngle))
                                / (largerAngle - smallerAngle));
        return resultRPM;
    }
}