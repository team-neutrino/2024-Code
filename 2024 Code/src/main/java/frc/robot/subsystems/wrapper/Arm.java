// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrapper;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.CalculateAngle;

public class Arm extends ArmSubsystem {
    CalculateAngle m_calculateAngle;
    public static double currentSimAngle;
    double motor_volts;

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic Sim_Angle = nt.getDoubleTopic("arm/sim_angle");
    DoubleTopic Encoder_Angle = nt.getDoubleTopic("arm/sim_encoder_angle");
    DoubleTopic m_realEncoderAngle = nt.getDoubleTopic("arm/real_encoder_angle");
    DoubleTopic Target_Angle = nt.getDoubleTopic("arm/target_angle");
    DoubleTopic Arm_Voltage = nt.getDoubleTopic("arm/motor_set_voltage");
    DoubleTopic theta_topic = nt.getDoubleTopic("arm/ theta");
    DoubleTopic radius_topic = nt.getDoubleTopic("arm/radius");
    final DoublePublisher simAnglePub;
    final DoublePublisher encoderAnglePub;
    final DoublePublisher targetAnglePub;
    final DoublePublisher motorVoltagePub;
    final DoublePublisher eAnglePub;
    final DoublePublisher radiPublisher;
    final DoublePublisher thetaPublisher;

    public Arm(CalculateAngle calculateAngle) {
        m_calculateAngle = calculateAngle;

        simAnglePub = Sim_Angle.publish();
        simAnglePub.setDefault(0.0);

        encoderAnglePub = Encoder_Angle.publish();
        encoderAnglePub.setDefault(0.0);

        targetAnglePub = Target_Angle.publish();
        targetAnglePub.setDefault(0.0);

        motorVoltagePub = Arm_Voltage.publish();
        motorVoltagePub.setDefault(0.0);

        eAnglePub = m_realEncoderAngle.publish();
        eAnglePub.setDefault(0.0);

        thetaPublisher = theta_topic.publish();
        thetaPublisher.setDefault(0.0);

        radiPublisher = radius_topic.publish();
        radiPublisher.setDefault(0.0);

    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        targetAnglePub.set(getTargetAngle(), now);
        encoderAnglePub.set(getArmAngleDegrees(), now);
        motorVoltagePub.set(motor_volts, now);
        eAnglePub.set(getArmAngleDegrees(), now);
        radiPublisher.set(m_calculateAngle.getRadius(), now);
        thetaPublisher.set(m_calculateAngle.getTheta(), now);
    }
}
// 688.78 is CG inertia Distance bwetwwn cg and axis is 15.17247438
// length in in 25.809 in
// min 29.5 degrees
// max 82 degrees
// 20lbs
// 9.0718474