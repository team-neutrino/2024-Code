// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class ShootWhilstSwervingMath {
    /**
     * Gets the robot's instantaneous x velocity with the forward direction being
     * the angle at which the robot would be perfectly facing the speaker.
     * 
     * @return The component of the robot's velocity that is tangential to the
     *         scoring angle.
     */
    public static double getSpeakerRelativeTangentialVelocity() {
        ChassisSpeeds x = SubsystemContainer.swerveSubsystem2.getChassisSpeeds();
        Rotation2d scoringAngle = new Rotation2d(Math.toRadians(SubsystemContainer.swerveSubsystem2.getYaw2()
                - SubsystemContainer.limelightSubsystem.getDistanceFromPrimaryTarget()));
        x = ChassisSpeeds.fromFieldRelativeSpeeds(x, scoringAngle);
        return x.vyMetersPerSecond;
    }
}
