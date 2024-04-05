// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class DynamicDebouncer {

    private double debounceTime;
    private double minErrorMargin;
    private double maxErrorMargin;

    private Debouncer debouncer;

    public DynamicDebouncer(double debounceTime, double minErrorMargin, double maxErrorMargin) {
        this.debounceTime = debounceTime;
        this.minErrorMargin = minErrorMargin;
        this.maxErrorMargin = maxErrorMargin;

        debouncer = new Debouncer(debounceTime, Debouncer.DebounceType.kRising);
    }

    public boolean calculate()
    {
        double r = SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getRadius();

        return debouncer.calculate((maxErrorMargin - minErrorMargin) )
    }

}
