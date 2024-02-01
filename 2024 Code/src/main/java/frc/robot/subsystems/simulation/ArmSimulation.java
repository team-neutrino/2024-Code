// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSimulation extends ArmSubsystem {
    double get_angle;
    DutyCycleEncoderSim m_armEncoderSim;

    Mechanism2d m_armMech = new Mechanism2d(10, 10);
    MechanismRoot2d m_root = m_armMech.getRoot("shoulder", 3, 3);
    MechanismLigament2d m_upperArm;

    SingleJointedArmSim m_armSim;
    double motor_volts;

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic Sim_Angle = nt.getDoubleTopic("arm/sim_angle");
    DoubleTopic Encoder_Angle = nt.getDoubleTopic("arm/encoder_angle");
    DoubleTopic Target_Angle = nt.getDoubleTopic("arm/target_angle");
    DoubleTopic Arm_Voltage = nt.getDoubleTopic("arm/motor_set_voltage");
    final DoublePublisher simAnglePub;
    final DoublePublisher encoderAnglePub;
    final DoublePublisher targetAnglePub;
    final DoublePublisher motorVoltagePub;
    CanSparkMaxPidSim pidSim;

    public ArmSimulation() {
        m_armEncoderSim = new DutyCycleEncoderSim(m_armEncoder);
        m_upperArm = m_root.append(new MechanismLigament2d("upperarm", 4, 0));
        m_armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 212.59, 6.9, 0.6555486, 0.507867133, 1.781293706, true,
                0.872665);
        SmartDashboard.putData("Arm", m_armMech);

        simAnglePub = Sim_Angle.publish();
        simAnglePub.setDefault(0.0);

        encoderAnglePub = Encoder_Angle.publish();
        encoderAnglePub.setDefault(0.0);

        targetAnglePub = Target_Angle.publish();
        targetAnglePub.setDefault(0.0);

        motorVoltagePub = Arm_Voltage.publish();
        motorVoltagePub.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_arm, DCMotor.getNEO(1));
        pidSim = new CanSparkMaxPidSim();
    }

    @Override
    public void simulationPeriodic() {
        // motor_volts = m_arm.getAppliedOutput() * m_arm.getBusVoltage();

        // System.out.println("this function is running");

        get_angle = m_armSim.getAngleRads() * (180 / Math.PI);
        System.out.println("current angle " + get_angle);
        motor_volts = pidSim.runPid(1, 0.0, 0.0, 0.0, getTargetAngle(), get_angle, 0.0, -13, 13);

        m_armSim.setInputVoltage(motor_volts);
        m_armSim.update(0.02);

        m_armEncoderSim.setAbsolutePosition(get_angle);
        m_upperArm.setAngle(get_angle);
        simAnglePub.set(m_upperArm.getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();
        targetAnglePub.set(getTargetAngle(), NetworkTablesJNI.now());
        encoderAnglePub.set(m_armEncoder.getAbsolutePosition(), NetworkTablesJNI.now());
        motorVoltagePub.set(motor_volts, NetworkTablesJNI.now());
    }
}
// 688.78 is CG inertia Distance bwetwwn cg and axis is 15.17247438
// length in in 25.809 in
// min 29.5 degrees
// max 82 degrees
// 20lbs
// 9.0718474