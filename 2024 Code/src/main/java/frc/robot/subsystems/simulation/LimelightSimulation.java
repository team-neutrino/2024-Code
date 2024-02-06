package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class LimelightSimulation {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic simTx = nt.getDoubleTopic("shooter/tx");
    DoubleTopic simTid = nt.getDoubleTopic("shooter/tid");
    DoubleTopic simTy = nt.getDoubleTopic("shooter/ty");
    DoubleTopic simTv = nt.getDoubleTopic("shooter/tv");

    DoublePublisher simTx_pub;
    DoublePublisher simTid_pub;
    DoublePublisher simTy_pub;
    DoublePublisher simTv_pub;

    DoubleSubscriber simTx_sub;
    DoubleSubscriber simTid_sub;
    DoubleSubscriber simTy_sub;
    DoubleSubscriber simTv_sub;
    public LimelightSimulation(){
    simTx_pub = simTx.publish();
        simTx_pub.setDefault(0);
        simTx_sub = simTx.subscribe(0, PubSubOption.sendAll(true));

        simTid_pub = simTid.publish();
        simTid_pub.setDefault(0);
        simTid_sub = simTid.subscribe(0, PubSubOption.sendAll(true));

        simTy_pub = simTy.publish();
        simTy_pub.setDefault(0);
        simTy_sub = simTy.subscribe(0, PubSubOption.sendAll(true));

        simTv_pub = simTv.publish();
        simTv_pub.setDefault(0);
        simTv_sub = simTv.subscribe(0, PubSubOption.sendAll(true));
    }
    public double GetId() {
        return simTid_sub.get();
    }

    public double GetTx() {
        return simTx_sub.get();
    }

    public double GetTy() {
        return simTy_sub.get();
    }

    public double GetTv() {
        return simTv_sub.get();
    }


}
