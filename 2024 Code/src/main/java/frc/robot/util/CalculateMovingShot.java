// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CalculateMovingShot {

    private SwerveSubsystem m_swerveSubsystem = SubsystemContainer.swerveSubsystem;

    /**
     * Uses the helper methods calculateAdjutedRadius and calculateAdjustedTheta to
     * create a PolarCoord representing the location to use when calculating an
     * interpolation shot.
     * 
     * @return The robot's adjusted position, accounting for constant movement
     *         parallel to the speaker.
     */
    public PolarCoord calculateAdjustedPos() {
        double r = m_swerveSubsystem.GetSpeakerToRobot().getRadius();
        double theta = m_swerveSubsystem.GetSpeakerToRobot().getTheta();
        double robotSpeed = m_swerveSubsystem.getDriveMotorSpeed();

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
