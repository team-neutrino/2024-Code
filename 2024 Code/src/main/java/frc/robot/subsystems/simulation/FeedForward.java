// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

/** Add your docs here. */
public class FeedForward {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic simFFK = nt.getDoubleTopic("arm/FFk");
    DoublePublisher simFFK_pub;
    DoubleSubscriber simFFK_sub;

    public FeedForward(double k) {

        simFFK_pub = simFFK.publish();
        simFFK_pub.setDefault(k);
        simFFK_sub = simFFK.subscribe(k, PubSubOption.sendAll(true));

    }

    public double getK() {
        return simFFK_sub.get();
    }

}
