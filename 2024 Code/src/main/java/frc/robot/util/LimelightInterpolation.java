// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.TreeMap;

/** Add your docs here. */
public class LimelightInterpolation {

    TreeMap<Double, Double> map = new TreeMap<>();

    public LimelightInterpolation() {
        map.put(20.52, 3.0);
        map.put(33.0, 5.0);
        map.put(55.0, 6.5);
    }

    public double calculateAngle(double inputAngle) {
    Double largerAngle = map.higherKey(inputAngle);
    Double smallerAngle = map.lowerKey(inputAngle);

    if (largerAngle == null)
    {
        largerAngle = map.lastKey();
    }
    if (smallerAngle == null)
    {
        smallerAngle = map.firstKey();
    }

    double out = (map.get(smallerAngle) * (largerAngle - inputAngle)
                + map.get(largerAngle) * (inputAngle - smallerAngle))
                / (largerAngle - smallerAngle);

    }

    return out;
}
