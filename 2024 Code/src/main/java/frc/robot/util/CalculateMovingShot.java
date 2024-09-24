// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShootWhilstSwerveConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Utility class that does math for shooting whilst swerving.
 */
public class CalculateMovingShot {

    /**
     * Anti-interpolation equation. Arm position is on the y-axis and is
     * measured in degrees, distance is radial from the speaker, measured in meters,
     * and on the x-axis. This solution is "laser-pointer," meaning that it does not
     * account for gravity.
     * 
     * Equation should solve: given an immobile and speaker-facing discrete robot
     * position within the firing range of a 4000 rpm shooter, input a radial
     * distance from the speaker and output an arm angle that will score.
     * 
     * With the above equation, shoot whilst swerving would be solved by simply
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
     */
    private static double AntiInterpolationEquation(double radialDist) {
        // wing is 231.2" or 5.87 m
        // Suboofer point is (0, -10) [in degrees above horizontal: (0, 28.92)],
        // preliminary check for equation.

        return Math.atan(1.30827 / radialDist + 1.2);
    }

    /**
     * Gets then modifies the output of the anti interpolation equation to be an arm
     * angle instead of an angle above the horizontal. This is needed because the
     * arm encoder only measures the angle of the bar that pivots the (angled)
     * shooter.
     * 
     * @param radialDist The shortest distance between the robot and the speaker.
     * @return The value to use as a reference for the arm.
     */
    public static double getArmAngle(double radialDist) {
        return AntiInterpolationEquation(radialDist) + ShootWhilstSwerveConstants.ARM_ANGLE_CONVERSION;
    }

    /**
     * Adjusts the given angle for robot speed. NOTE: speed must be RADIAL!! Meaning
     * it is only the speed at which the robot is approaching or receding from the
     * speaker, NO LATERAL SPEED should be included!
     * 
     * @param robotSpeed The robot's current RADIAL speed.
     * @param radialDist The robot's current radial distance from the speaker.
     * @return The adjusted angle that will only work for the given robot speed.
     */
    public static double adjustArmForMovement(double robotSpeed, double radialDist) {
        double metersAhead = robotSpeed * ShootWhilstSwerveConstants.MOVEMENT_ADJUSTMENT_TIME;
        return AntiInterpolationEquation(metersAhead + radialDist);
    }
}
