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
    DoubleTopic intakeWheelSimSpeed_topic = inst.getDoubleTopic("Intake/sim_speed");
    DoubleTopic intakeWheelEncSpeed_topic = inst.getDoubleTopic("Intake/encoder_speed");
    BooleanTopic beambreakStatus_topic = inst.getBooleanTopic("Intake/beam_break");
    DoubleTopic indexSpeed_topic = inst.getDoubleTopic("Index/target_speed");
    final DoublePublisher intakeWheelSimSpeed_pub;
    final DoublePublisher intakeWheelEncSpeed_pub;
    final DoublePublisher indexSpeed_pub;

    final BooleanPublisher beambreakStatus_pub;

    DoubleTopic indexWheelSimSpeed_topic = inst.getDoubleTopic("Index/sim_speed");
    DoubleTopic indexWheelEncSpeed_topic = inst.getDoubleTopic("Index/encoder_speed");
    final DoublePublisher indexWheelSimSpeed_pub;
    final DoublePublisher indexWheelEncSpeed_pub;

    public Intake() {

        intakeWheelSimSpeed_pub = intakeWheelSimSpeed_topic.publish();
        intakeWheelSimSpeed_pub.setDefault(0.0);

        intakeWheelEncSpeed_pub = intakeWheelEncSpeed_topic.publish();
        intakeWheelEncSpeed_pub.setDefault(0.0);

        indexWheelSimSpeed_pub = indexWheelSimSpeed_topic.publish();
        indexWheelSimSpeed_pub.setDefault(0.0);

        indexSpeed_pub = indexSpeed_topic.publish();
        indexSpeed_pub.setDefault(0.0);

        indexWheelEncSpeed_pub = indexWheelEncSpeed_topic.publish();
        indexWheelEncSpeed_pub.setDefault(0.0);

        beambreakStatus_pub = beambreakStatus_topic.publish();
        beambreakStatus_pub.setDefault(false);

    }

    public void periodic() {
        super.periodic();
        intakeWheelEncSpeed_pub.set(m_intakeEncoder.getVelocity(), NetworkTablesJNI.now());
        indexWheelEncSpeed_pub.set(m_indexEncoder.getVelocity(), NetworkTablesJNI.now());
        beambreakStatus_pub.set(isBeamBrokenIntake());
        indexSpeed_pub.set(getIndexVoltage());
    }
}