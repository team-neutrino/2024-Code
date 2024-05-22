// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PDH {
    PowerDistribution m_powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);

    public PDH() {
    }

    public double getCurrentFromChannel(int channel) {
        return m_powerDistributionHub.getCurrent(channel);
    }

    // public double getTotalCurrent() {
    // return m_powerDistributionHub.getTotalCurrent();
    // }
}
