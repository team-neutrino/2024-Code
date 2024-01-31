package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class PIDChangerSimulationShooter {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic simP = nt.getDoubleTopic("shooter/sim_P");
    DoubleTopic simI = nt.getDoubleTopic("shooter/sim_I");
    DoubleTopic simD = nt.getDoubleTopic("shooter/sim_D");
    DoubleTopic simFF = nt.getDoubleTopic("shooter/sim_FF");
    BooleanTopic simX = nt.getBooleanTopic("shooter/sim_PIDchangeapprove");

    DoublePublisher simP_pub;
    DoublePublisher simI_pub;
    DoublePublisher simD_pub;
    DoublePublisher simFF_pub;
    BooleanPublisher simX_pub;

    DoubleSubscriber simP_sub;
    DoubleSubscriber simI_sub;
    DoubleSubscriber simD_sub;
    DoubleSubscriber simFF_sub;
    BooleanSubscriber simX_sub;

    public PIDChangerSimulationShooter(double p, double i, double d, double ff, boolean x) {

        simP_pub = simP.publish();
        simP_pub.setDefault(p);
        simP_sub = simP.subscribe(p, PubSubOption.sendAll(true));

        simI_pub = simI.publish();
        simI_pub.setDefault(i);
        simI_sub = simI.subscribe(i, PubSubOption.sendAll(true));

        simD_pub = simD.publish();
        simD_pub.setDefault(d);
        simD_sub = simD.subscribe(d, PubSubOption.sendAll(true));

        simFF_pub = simFF.publish();
        simFF_pub.setDefault(ff);
        simFF_sub = simFF.subscribe(ff, PubSubOption.sendAll(true));

        simX_pub = simX.publish();
        simX_pub.setDefault(x);
        simX_sub = simX.subscribe(x, PubSubOption.sendAll(true));

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

    public double GetFF() {
        return simFF_sub.get();
    }

    public boolean simPIDChangeApprove() {
        boolean x = simX_sub.get();
        simX_pub.set(false);
        return x;
    }

}
