package frc.robot.subsystems.wrapper;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends IntakeSubsystem {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    BooleanTopic beambreakStatus_topic = inst.getBooleanTopic("Intake/beam_break");
    DoubleTopic indexSpeed_topic = inst.getDoubleTopic("Index/target_speed");

    final DoublePublisher indexSpeed_pub;

    final BooleanPublisher beambreakStatus_pub;

    public Intake() {

        indexSpeed_pub = indexSpeed_topic.publish();
        indexSpeed_pub.setDefault(0.0);

        beambreakStatus_pub = beambreakStatus_topic.publish();
        beambreakStatus_pub.setDefault(false);

    }

    public void periodic() {
        super.periodic();
        beambreakStatus_pub.set(isBeamBrokenIntake());
        indexSpeed_pub.set(getIndexVoltage());
    }
}