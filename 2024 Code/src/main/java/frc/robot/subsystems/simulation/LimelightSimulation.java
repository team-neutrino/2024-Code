package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightSimulation extends LimelightSubsystem {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    
    DoubleTopic simTx = nt.getDoubleTopic("limelight/tx");
    DoubleTopic simTid = nt.getDoubleTopic("limelight/tid");
    DoubleTopic simTy = nt.getDoubleTopic("limelight/ty");
    BooleanTopic simTv = nt.getBooleanTopic("limelight/tv");
    BooleanTopic simApprove = nt.getBooleanTopic("limelight/approve");

    DoublePublisher simTx_pub;
    DoublePublisher simTid_pub;
    DoublePublisher simTy_pub;
    BooleanPublisher simTv_pub;
    BooleanPublisher simApprove_pub;

    DoubleSubscriber simTx_sub;
    DoubleSubscriber simTid_sub;
    DoubleSubscriber simTy_sub;
    BooleanSubscriber simTv_sub;
    BooleanSubscriber simApprove_sub;

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
        simTv_pub.setDefault(false);
        simTv_sub = simTv.subscribe(false, PubSubOption.sendAll(true));

        simApprove_pub = simApprove.publish();
        simApprove_pub.setDefault(false);
        simApprove_sub = simApprove.subscribe(false, PubSubOption.sendAll(true)); 
    }

    public Boolean getApprove(){
        return simApprove_sub.get();
    }
    public double GetId() {
        if(getApprove()){
            return simTid_sub.get();
        }
        return super.getID();
    }

    public double GetTx() {
       if(getApprove()){
            return simTx_sub.get();        
        } 
        return super.getTx();
    }

    public double GetTy() {
         if(getApprove()){
            return simTy_sub.get();
        } 
            return super.getTy();
    }

    public boolean GetTv() {
        if(getApprove()){
            return simTv_sub.get();
        } 
        return super.getTv();
    }

    public void periodic(){
        super.periodic();
        simTv_pub.set(GetTv());
        simTy_pub.set(GetTy());
        simTx_pub.set(GetTx());
        simTid_pub.set(GetId());



    }


}
