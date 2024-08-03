package frc.robot.util;

import java.util.TreeMap;

public class CalculateP {

    TreeMap<Double, Double> m_map;

    public CalculateP(TreeMap<Double, Double> p_map) {
        m_map = p_map;
    }

    public double InterpolateP(double p_error) {
        double smallerError = m_map.firstKey();
        double largerError = m_map.firstKey();
        double resultP = 0;
        double error = p_error;

        if (error <= m_map.firstKey()) {
            return m_map.get(m_map.firstKey());
        } else if (error >= m_map.lastKey()) {
            return m_map.get(m_map.lastKey());
        } else {
            for (Double a : m_map.keySet()) {
                if (a >= error) {
                    largerError = a;
                    break;
                } else {
                    smallerError = a;
                }
            }
        }
        resultP = m_map.get(smallerError)
                + ((error - smallerError))
                        * ((m_map.get(largerError) - m_map.get(smallerError))
                                / (largerError - smallerError));
        return resultP;
    }
}