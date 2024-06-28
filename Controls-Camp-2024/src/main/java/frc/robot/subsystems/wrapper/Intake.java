package frc.robot.subsystems.wrapper;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends IntakeSubsystem {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    BooleanTopic beambreakStatus_topic = inst.getBooleanTopic("Intake/beam_break");
    BooleanTopic indexBeamBreakStatus_topic = inst.getBooleanTopic("Index/beam_break");
    DoubleTopic indexSpeed_topic = inst.getDoubleTopic("Index/target_speed");

    final DoublePublisher indexSpeed_pub;

    final BooleanPublisher beambreakStatus_pub;
    final BooleanPublisher indexBeamBreakStatus_pub;

    public Intake() {

        indexSpeed_pub = indexSpeed_topic.publish();
        indexSpeed_pub.setDefault(0.0);

        beambreakStatus_pub = beambreakStatus_topic.publish();
        beambreakStatus_pub.setDefault(false);

        indexBeamBreakStatus_pub = indexBeamBreakStatus_topic.publish();
        indexBeamBreakStatus_pub.setDefault(false);

    }

    public void periodic() {
        super.periodic();
        beambreakStatus_pub.set(isBeamBrokenIntake());
        indexBeamBreakStatus_pub.set(isBeamBrokenIndex());
        indexSpeed_pub.set(getIndexVoltage());
    }
}