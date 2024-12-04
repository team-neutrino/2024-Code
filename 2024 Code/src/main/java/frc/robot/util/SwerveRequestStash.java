// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveRequestStash {
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(SwerveConstants.MaxSpeed * 0.1)
                        // .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.06)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        public static final FieldCentricFacingAngle drive1 = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(SwerveConstants.MaxSpeed * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        public static void configurePIDControllers() {
                drive1.HeadingController = new PhoenixPIDController(1.2, 0, 0);
        }
}
