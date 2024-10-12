// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.ShooterConstants;;

/**
 * Utility class that does math for shooting whilst swerving.
 */
public class AntiInterpolationCalculation {

    public static void main(String[] args) {
        for (int radial = 0; radial < 5; radial++) {
            System.out.println("For distance " + radial + " m, arm angle is: " + getArmAngle(radial));
        }
    }

    /**
     * Anti-interpolation equation. Arm position is on the y-axis and is
     * measured in degrees, distance is on the x-axis, radial from the speaker, and
     * measured in meters, This solution is "laser-pointer," meaning that it does
     * not account for gravity.
     * 
     * With this equation, shoot whilst swerving would be solved by simply
     * scheduling the target arm angle further from the current radial distance
     * depending on the robot velocity. Since the problem is now being treated in a
     * robot oriented fashion (auto-align is assumed to continually and perfectly
     * update angle to speaker), side-to-side movement (VsubY) is insignificant.
     * 
     * Robot velocity is assumed to be constant as the robot accelerates to its
     * desired speed almost instantly. If acceleration becomes necessary to
     * implement in the future, an alternate solution of directly reading the
     * driver's stick inputs to see if movement in a direction will be requested to
     * the swerve may be easier than actually including acceleration in the
     * equation.
     * 
     * In the future, a slight "flick" in robot orientation right before the shot
     * may be necessary if the auto-align cannot keep up with robot movement.
     * 
     * @param radialDist The shortest distance between the robot and the speaker.
     * @return An angle in radians above the horizontal which will score if the note
     *         is shot from the ground level.
     */
    private static double AntiInterpolationEquation(double radialDist, double initialHeight) {
        // wing is 231.2" or 5.87 m

        // Suboofer point is (0, -10) [in degrees above horizontal: (0, 48.92)],
        // preliminary check for equation.

        // Function is negated becuase as arm angle decreases, angle above
        // horizontal/actual shot angle INCREASES. This allows for a more intuitive
        // conversion to arm angle.

        // Ideal speaker impact point: 1.8907125 m

        // fudge factor is currently 0 for testing purposs but median value from
        // prevoius testing shows -7 degrees or .122173 radians to be most accurate.
        return -Math.atan((1.8907 - initialHeight) / (radialDist + 1.2)) + 0;
    }

    /**
     * The initial height of the note when leaving the robot changes depending on
     * the angle of the arm. To acount for this, the following equation has been
     * made.
     * 
     * Length of pivot: .343 m, initial height WHEN ARM IS FULLY DOWN (-27): .508 m
     * 
     * sin(antiInterpolation output) = (initial shot height + 27) / (.343)
     * initial shot height = .343 * sin(antiInterpolation output + 27)
     * add initial shot height at fully down position of .508 and convert 27 degrees
     * to radians: 3pi/20 to get final equation:
     * 
     * initial shot height = .343 * sin((antiInterpolation output) + 3pi/20) + .508
     * 
     * @param armAngle The output of the anti-interpolation equation.
     * @return The initial height of the note when it is shot.
     */
    private static double adjustForInitialShotHeight(double armAngle) {
        return (.343 * Math.sin(armAngle + ((3 * Math.PI) / 20))) + .508;
    }

    /**
     * Gets then modifies the output of the anti interpolation equation so that it
     * is usable as an input to the arm subsystem.
     * 
     * Calls antiInterpolationEquation twice, once to get the angle above horizontal
     * without taking into account initial shot height, then a second time, this
     * time subtracting the initial shot height from the ideal speaker impact point.
     * This does NOT give a perfect answer, more iterations would be better, but
     * this is most likely good enough.
     * 
     * The second return of the antiInterpolationEquation is then modified to be an
     * arm angle in degrees instead of a radian angle above the horizontal. This is
     * needed because the arm encoder only measures the angle of the bar that pivots
     * the shooter, and the shooter is not angled parallel to that bar.
     * 
     * @param radialDist The shortest distance between the robot and the speaker.
     * @return The "arm angle" (in degrees) to use as a reference for the arm.
     */
    public static double getArmAngle(double radialDist) {
        double initialValue = AntiInterpolationEquation(radialDist, 0);
        double radianFinalValue = AntiInterpolationEquation(radialDist, adjustForInitialShotHeight(initialValue));

        return (radianFinalValue * (180 / Math.PI)) + ShooterConstants.ARM_ANGLE_CONVERSION;
    }
}
