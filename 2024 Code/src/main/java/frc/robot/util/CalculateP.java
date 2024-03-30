package frc.robot.util;

import java.util.TreeMap;

import frc.robot.subsystems.ArmSubsystem;

public class CalculateP {

    TreeMap<Double, Double> m_calculateP = new TreeMap<Double, Double>();
    ArmSubsystem m_armSubsystem;

    public CalculateP() {
        m_armSubsystem = SubsystemContainer.armSubsystem;
        m_calculateP.put(2.0, 0.022);
        m_calculateP.put(7.0, 0.04);
    }

    public double InterpolateP() {
        double smallerError = 0;
        double largerError = 0;
        double resultP = 0;
        double error = m_armSubsystem.getError();

        if (error <= m_calculateP.firstKey()) {
            return m_calculateP.get(m_calculateP.firstKey());
        } else if (error >= m_calculateP.lastKey()) {
            return m_calculateP.get(m_calculateP.lastKey());
        } else {
            for (Double a : m_calculateP.keySet()) {
                if (a >= error) {
                    largerError = a;
                    break;
                } else {
                    smallerError = a;
                }
            }
        }
        resultP = m_calculateP.get(smallerError)
                + ((error - smallerError))
                        * ((m_calculateP.get(largerError) - m_calculateP.get(smallerError))
                                / (largerError - smallerError));
        return resultP;
    }
}