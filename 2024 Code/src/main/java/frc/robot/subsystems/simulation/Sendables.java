// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import java.util.ArrayList;
import java.util.concurrent.atomic.DoubleAccumulator;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Sendables {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    ArrayList<DoubleTopic> pdhChannels_topic = new ArrayList<DoubleTopic>();
    ArrayList<DoublePublisher> pdhChannels_pub = new ArrayList<DoublePublisher>();

    public Sendables() {
        updateCurrentList();
    }

    public void updateCurrentList() {
        for (int i = 0; i >= 20; i++) {
            pdhChannels_topic.add(inst.getDoubleTopic("Sendables/PDH/Channel " + i + "Current"));
            DoubleTopic m_topic = pdhChannels_topic.get(i);
            DoublePublisher m_publish = pdhChannels_pub.get(i);
            m_publish = m_topic.publish();
            m_publish.setDefault(0.0);
        }
    }

    public void simulationPeriodic() {
        System.out.println(pdhChannels_topic.get(0));
    }
}
