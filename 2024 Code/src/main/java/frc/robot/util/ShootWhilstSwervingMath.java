// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShootWhilstSwervingConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class ShootWhilstSwervingMath {
    /**
     * Gets the robot's instantaneous side-to-side velocity with the forward
     * direction being the angle at which the robot would be perfectly facing the
     * speaker.
     * 
     * @return The component of the robot's velocity that is tangential to the
     *         scoring angle.
     */
    public static double getSpeakerRelativeTangentialVelocity() {
        ChassisSpeeds x = SubsystemContainer.swerveSubsystem2.getChassisSpeeds();
        Rotation2d scoringAngle = Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem2.getYaw2()
                - SubsystemContainer.limelightSubsystem.getDistanceFromPrimaryTarget());
        x = ChassisSpeeds.fromFieldRelativeSpeeds(x, scoringAngle);
        return x.vyMetersPerSecond;
    }

    /**
     * Gets the robot's instantaneous forward/backward velocity with the forward
     * direction being the angle at which the robot would be perfectly facing the
     * speaker.
     * 
     * @return The portion of the robot's velocity that is directly forwards or
     *         backwards from the speaker in m/s.
     */
    public static double getSpeakerRelativeRadialVelocity() {
        ChassisSpeeds x = SubsystemContainer.swerveSubsystem2.getChassisSpeeds();
        Rotation2d scoringAngle = Rotation2d.fromDegrees(SubsystemContainer.swerveSubsystem2.getYaw2()
                - SubsystemContainer.limelightSubsystem.getDistanceFromPrimaryTarget());
        x = ChassisSpeeds.fromFieldRelativeSpeeds(x, scoringAngle);
        return x.vxMetersPerSecond;
    }

    /**
     * Calculates the target angle to autoalign to, correcting as necessary for
     * movement. This angle is generated with the assumption that the robot is
     * already directly facing the speaker, so it can be performed without
     * limelight, but must be summed with the existing speaker-to-robot differential
     * to get an angle that can be used to autoalign.
     * 
     * @return The angle to autoalign to in radians.
     */
    private static double getMovementCorrectionAngle() {
        return Math.atan(getSpeakerRelativeTangentialVelocity()
                / (ShootWhilstSwervingConstants.NOTE_SPEED - getSpeakerRelativeRadialVelocity()));
    }

    /**
     * Calculates the rad/s value to plug into a swerverequest, accounting for
     * movement and normal offset.
     * 
     * @return The angle to use in moving auto align in rad/s.
     */
    public static double movingOffsetToOmega() {
        // getMovementCorrectionAngle returns the angle that we need to go to, so
        // convert this to an offset for consistency with the limelight returned tx
        double movementCorrectedDifferential = Math.toRadians(SubsystemContainer.swerveSubsystem2.getYaw2())
                - getMovementCorrectionAngle();
        double totalDifferential = movementCorrectedDifferential
                + Math.toRadians(SubsystemContainer.limelightSubsystem.getOffsetAngleFromTag());
        // after the following calculation, the value should be between 0 and 1
        totalDifferential /= ShootWhilstSwervingConstants.MOVING_AUTOALIGN_FEEDFOWARD;
        return totalDifferential * SwerveConstants.MaxAngularRate * .5;
    }
}
