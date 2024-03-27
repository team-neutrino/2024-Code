package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PDH;

public class NetworkTables extends SubsystemBase {
    PDH m_Pdh = new PDH();
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic port1_topic = inst.getDoubleTopic("PDH/port1");
    DoubleTopic port2_topic = inst.getDoubleTopic("PDH/port2");
    DoubleTopic port3_topic = inst.getDoubleTopic("PDH/port3");
    DoubleTopic port4_topic = inst.getDoubleTopic("PDH/port4");
    DoubleTopic port5_topic = inst.getDoubleTopic("PDH/port5");
    DoubleTopic port6_topic = inst.getDoubleTopic("PDH/port6");
    DoubleTopic port7_topic = inst.getDoubleTopic("PDH/port7");
    DoubleTopic port8_topic = inst.getDoubleTopic("PDH/port8");
    DoubleTopic port9_topic = inst.getDoubleTopic("PDH/port9");
    DoubleTopic port10_topic = inst.getDoubleTopic("PDH/port10");
    DoubleTopic port11_topic = inst.getDoubleTopic("PDH/port11");
    DoubleTopic port12_topic = inst.getDoubleTopic("PDH/port12");
    DoubleTopic port13_topic = inst.getDoubleTopic("PDH/port13");
    DoubleTopic port14_topic = inst.getDoubleTopic("PDH/port14");
    DoubleTopic port15_topic = inst.getDoubleTopic("PDH/port15");
    DoubleTopic port16_topic = inst.getDoubleTopic("PDH/port16");
    DoubleTopic port17_topic = inst.getDoubleTopic("PDH/port17");
    DoubleTopic port18_topic = inst.getDoubleTopic("PDH/port18");
    DoubleTopic port19_topic = inst.getDoubleTopic("PDH/port19");
    DoubleTopic port20_topic = inst.getDoubleTopic("PDH/port20");
    DoubleTopic port21_topic = inst.getDoubleTopic("PDH/port21");
    DoubleTopic port22_topic = inst.getDoubleTopic("PDH/port22");
    DoubleTopic port23_topic = inst.getDoubleTopic("PDH/port23");
    DoubleTopic total_current = inst.getDoubleTopic("PDH/total");
    final DoublePublisher[] ports_pub = new DoublePublisher[23];
    final DoublePublisher totalCurrent;

    public NetworkTables() {
        ports_pub[0] = port1_topic.publish();
        ports_pub[0].setDefault(0.0);

        ports_pub[1] = port2_topic.publish();
        ports_pub[1].setDefault(0.0);

        ports_pub[2] = port3_topic.publish();
        ports_pub[2].setDefault(0.0);

        ports_pub[3] = port4_topic.publish();
        ports_pub[3].setDefault(0.0);

        ports_pub[4] = port5_topic.publish();
        ports_pub[4].setDefault(0.0);

        ports_pub[5] = port6_topic.publish();
        ports_pub[5].setDefault(0.0);

        ports_pub[6] = port7_topic.publish();
        ports_pub[6].setDefault(0.0);

        ports_pub[7] = port8_topic.publish();
        ports_pub[7].setDefault(0.0);

        ports_pub[8] = port9_topic.publish();
        ports_pub[8].setDefault(0.0);

        ports_pub[9] = port10_topic.publish();
        ports_pub[9].setDefault(0.0);

        ports_pub[10] = port11_topic.publish();
        ports_pub[10].setDefault(0.0);

        ports_pub[11] = port12_topic.publish();
        ports_pub[11].setDefault(0.0);

        ports_pub[12] = port13_topic.publish();
        ports_pub[12].setDefault(0.0);

        ports_pub[13] = port14_topic.publish();
        ports_pub[13].setDefault(0.0);

        ports_pub[14] = port15_topic.publish();
        ports_pub[14].setDefault(0.0);

        ports_pub[15] = port16_topic.publish();
        ports_pub[15].setDefault(0.0);

        ports_pub[16] = port17_topic.publish();
        ports_pub[16].setDefault(0.0);

        ports_pub[17] = port18_topic.publish();
        ports_pub[17].setDefault(0.0);

        ports_pub[18] = port19_topic.publish();
        ports_pub[18].setDefault(0.0);

        ports_pub[19] = port20_topic.publish();
        ports_pub[19].setDefault(0.0);

        ports_pub[20] = port21_topic.publish();
        ports_pub[20].setDefault(0.0);

        ports_pub[21] = port22_topic.publish();
        ports_pub[21].setDefault(0.0);

        ports_pub[22] = port23_topic.publish();
        ports_pub[22].setDefault(0.0);

        totalCurrent = total_current.publish();
        totalCurrent.setDefault(0.0);

    }

    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        totalCurrent.set(m_Pdh.getTotalCurrent());
    }
}