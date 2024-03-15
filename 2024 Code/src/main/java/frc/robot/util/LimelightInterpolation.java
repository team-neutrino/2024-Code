// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.TreeMap;

/** Add your docs here. */
public class LimelightInterpolation {

    private TreeMap<Double, Double> map = new TreeMap<>();

    public LimelightInterpolation() {

    }

    public double interpolateAngle(double inputAngle) {
        double smallerAngle = 0;
        double largerAngle = 0;
        double resultRPM = 0;
        // double angle = m_armSubsystem.getArmAngleDegrees();

        // if (angle <= m_angleRPMData.firstKey()) {
        // return m_angleRPMData.get(m_angleRPMData.firstKey());
        // } else if (angle >= m_angleRPMData.lastKey()) {
        // return m_angleRPMData.get(m_angleRPMData.lastKey());
        // } else {
        // for (Double a : m_angleRPMData.keySet()) {
        // if (a >= angle) {
        // largerAngle = a;
        // break;
        // } else {
        // smallerAngle = a;
        // }
        // }
        // }
        // resultRPM = m_angleRPMData.get(smallerAngle)
        // + ((angle - smallerAngle))
        // * ((m_angleRPMData.get(largerAngle) - m_angleRPMData.get(smallerAngle))
        // / (largerAngle - smallerAngle));
        // return resultRPM;
        // }
        return 0;
    }
}
