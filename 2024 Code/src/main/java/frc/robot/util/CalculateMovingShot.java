// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

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
     * The subwoofer position (x = 0, y = -10) will be the (0,0) of this equation,
     * meaning: output must be translated +10 to achieve the correct arm angle,
     * x = 0 represents the robot flushed against the subwoofer, and y = 0
     * represents the subwoofer shooting position, which is -10 degrees.
     * 
     * Equation should solve: given an immobile and speaker-facing discrete
     * (varying) robot position within the firing range of a static 4000 rpm
     * shooter, plug in the radial distance from the speaker and always make the
     * shot.
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
     */
    public void AntiInterpolationEquation() {
        double intakePos = -27;
        double subwooferPos = -10;
        // x = 0, y = -10
        //
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
