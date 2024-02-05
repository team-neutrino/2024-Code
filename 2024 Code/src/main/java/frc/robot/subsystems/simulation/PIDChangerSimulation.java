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
public class PIDChangerSimulation {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic simP = nt.getDoubleTopic("arm/simP");
    DoubleTopic simI = nt.getDoubleTopic("arm/sim_I");
    DoubleTopic simD = nt.getDoubleTopic("arm/sim__D");

    DoublePublisher simP_pub;
    DoublePublisher simI_pub;
    DoublePublisher simD_pub;

    DoubleSubscriber simP_sub;
    DoubleSubscriber simI_sub;
    DoubleSubscriber simD_sub;

    public PIDChangerSimulation(double p, double i, double d) {

        simP_pub = simP.publish();
        simP_pub.setDefault(p);
        simP_sub = simP.subscribe(p, PubSubOption.sendAll(true));

        simI_pub = simI.publish();
        simI_pub.setDefault(i);
        simI_sub = simI.subscribe(i, PubSubOption.sendAll(true));

        simD_pub = simD.publish();
        simD_pub.setDefault(d);
        simD_sub = simD.subscribe(d, PubSubOption.sendAll(true));

    }

    public double GetP() {
        return simP_sub.get();
    }

    public double GetI() {
        return simI_sub.get();
    }

    public double GetD() {
        return simD_sub.get();
    }

}
