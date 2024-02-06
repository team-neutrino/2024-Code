// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class Limiter {
    /**
     * Scale desired input between min and max. Input value should be between [-1, 1]
     * @param in input value
     * @param min minimum possible value that is desired for output
     * @param max maximum possible value that is desired for output
     * @return A value that is scaled for the min and max values
     */
    public static double scale(double in, double min, double max){
        return (max-min)*(in+1)/(2)+min;
    }

    public static double deadzone(double in, double bound){
        if(Math.abs(in)>bound) {
            return in;
        }
        else {
            return 0;
        }
    }
}

