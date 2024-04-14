package frc.robot.subsystems.wrapper;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooter extends ShooterSubsystem {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic wheel_sim_speed_topic = inst.getDoubleTopic("shooter/sim_speed");
    DoubleTopic wheel_enc_speed_topic = inst.getDoubleTopic("shooter/encoder_speed");
    DoubleTopic wheel_target_speed_topic = inst.getDoubleTopic("shooter/target_speed");
    DoubleTopic wheel_sim_speed_topic2 = inst.getDoubleTopic("shooter/sim_speed2");
    DoubleTopic wheel_enc_speed_topic2 = inst.getDoubleTopic("shooter/encoder_speed2");
    DoubleTopic wheel_target_speed_topic2 = inst.getDoubleTopic("shooter/target_speed2");
    final DoublePublisher wheel_sim_speed_pub;
    final DoublePublisher wheel_speed_pub;
    final DoublePublisher wheel_target_speed_pub;
    final DoublePublisher wheel_sim_speed_pub2;
    final DoublePublisher wheel_target_speed_pub2;

    public Shooter() {

        wheel_sim_speed_pub = wheel_sim_speed_topic.publish();
        wheel_sim_speed_pub.setDefault(0.0);

        wheel_speed_pub = wheel_enc_speed_topic.publish();
        wheel_speed_pub.setDefault(0.0);

        wheel_target_speed_pub = wheel_target_speed_topic.publish();
        wheel_target_speed_pub.setDefault(0.0);

        wheel_sim_speed_pub2 = wheel_sim_speed_topic2.publish();
        wheel_sim_speed_pub2.setDefault(0.0);

        wheel_target_speed_pub2 = wheel_target_speed_topic2.publish();
        wheel_target_speed_pub2.setDefault(0.0);
    }

    public void periodic() {
        super.periodic();
        wheel_speed_pub.set(getShooterRPM(), NetworkTablesJNI.now());
        wheel_target_speed_pub.set(getTargetRPM(), NetworkTablesJNI.now());
        wheel_target_speed_pub2.set(getTargetRPM(), NetworkTablesJNI.now());
    }
}
