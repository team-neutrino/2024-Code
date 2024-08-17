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
     * Quadratic anti-interpolation equation. Arm position is on the y-axis and is
     * measured in degrees, distance is radial from the speaker, measured in meters,
     * and on the x-axis.
     * 
     * The subwoofer position (x = 0, y = -10 [arm angle! needs to be converted to
     * angle above horizontal!]) hopefully will be the only predetermined point in
     * this equation. Everything else should be math.
     * 
     * Equation should solve: given an immobile and speaker-facing discrete
     * (varying) robot position within the firing range of a static 4000 rpm
     * shooter, plug in the radial distance from the speaker and get an arm
     * angle that will score.
     * 
     * With the above equation, "shoot whilst swerving" would be solved by simply
     * scheduling the target arm angle further from the current radial distance
     * depending on the robot speed (which is assumed to be constant). Since the
     * problem is now being treated in a robot oriented fashion (auto-align is
     * assumed to continually and perfectly update angle to speaker), side-to-side
     * movement (VsubY) is insignificant.
     * 
     * In the future, a slight "flick" in robot orientation right before the shot
     * may be necessary if the auto-align cannot keep up with robot movement.
     * 
     * @param radialDist The shortest distance between the robot and the speaker.
     */
    private static double AntiInterpolationEquation(double radialDist) {
        // x = 0, y = ??? is subwoofer shot point

        // approximate note height when leaving shooter: 30 in or .762 meters

        // max range: 19.433 degrees (NOT ARM ANGLE - 19.433 DEGREES
        // ABOVE HORIZONTAL SHOOTING ANGLE) gives apagee of exactly 1.8097 meters
        // (middle of speaker height) in .6212 sec, horizontal range of 10.712 m

        // 20.44 degrees above horizontal currently used in calculations/graph - it's
        // incorrect, based off of faulty 2.04 meter middle of speaker height, but good
        // enough for now (5.47, *translated to arm* 20.44 degrees) (3193 sec)

        // wing is 231.2" or 5.87 m

        // max range point: (5.47,???) need to translate 20.44 degrees above horizontal
        // to arm angle, then create equation based on those two points

        // guestimation: (5.47, 10), y = .66829x^2 - 10
        // exponential guestimation (very similar): (5.47, 10) 1.7447^x - 11

        // no dividing by zero here
        if (Math.abs(radialDist) < .1) {
            return ArmConstants.SUBWOOFER_ANGLE;
        }

        // "- 20" is conversion from degrees above horizontal to arm degrees (it's just
        // my guess at the angle between the bar around which the arm pivots and the
        // line which the note comes out at)
        return Math.atan(1.2049 / radialDist) - 20;

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
        return AntiInterpolationEquation(radialDist) - ShootWhilstSwerveConstants.ARM_ANGLE_CONVERSION;
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

    /**
     * Uses the helper methods calculateAdjutedRadius and calculateAdjustedTheta to
     * create a PolarCoord representing the location to use when calculating an
     * interpolation shot.
     * 
     * @return The robot's adjusted position, accounting for constant movement
     *         parallel to the speaker.
     */
    public PolarCoord calculateAdjustedPos() {
        double r = SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getRadius();
        double theta = SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getTheta();
        double robotSpeed = SubsystemContainer.swerveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond;

        double deltaX = (r / ShooterConstants.NOTE_SPEED) * robotSpeed;

        return new PolarCoord(r, calculateAdjustedTheta(r, theta, deltaX));
    }

    /**
     * Calculates the adjusted "alpha" value for auto aligning.
     * 
     * @param r      The distance from the robot to the speaker.
     * @param theta  The angle from the robot to the speaker.
     * @param deltaX The distance the robot will move based on r and the ring shot
     *               speed.
     * @return The angle to auto align to based on the above parameters.
     */
    private double calculateAdjustedTheta(double r, double theta, double deltaX) {
        double adjustedRadius = calculateAdjustedRadius(r, theta, deltaX);

        return Math.asin((deltaX * Math.sin(theta)) / adjustedRadius);
    }

    /**
     * Calculates the distance the robot WILL be from the speaker after a shot from
     * the current position hits the speaker (assuming constant velocity)
     * 
     * @param r      The distance from the robot to the speaker.
     * @param theta  The angle from the robot to the speaker.
     * @param deltaX The distance the robot will move based on r and the ring shot
     * @return The distance from the speaker when a shot from the current location
     *         would hit the speaker.
     */
    private double calculateAdjustedRadius(double r, double theta, double deltaX) {
        return Math.sqrt((Math.pow(r, 2) + Math.pow(deltaX, 2)) - (2 * r * deltaX * Math.cos(theta)));
    }
}
