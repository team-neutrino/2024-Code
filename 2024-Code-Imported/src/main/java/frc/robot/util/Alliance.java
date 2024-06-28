// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class Alliance {

    public boolean isRedAlliance() {
        boolean isRed = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red;
        }
        return isRed;
    }
}
