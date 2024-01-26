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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/** Add your docs here. */
public class ArmSimulation extends ArmSubsystem {
    static double get_angle;

    Mechanism2d m_armMech = new Mechanism2d(10, 10);
    MechanismRoot2d m_root = m_armMech.getRoot("shoulder", 3, 3);
    MechanismLigament2d m_upperArm;

    SingleJointedArmSim m_armSim;

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic Sim_Angle = nt.getDoubleTopic("arm/sim_angle");
    DoubleTopic Encoder_Angle = nt.getDoubleTopic("arm/encoder_angle");
    DoubleTopic Target_Angle = nt.getDoubleTopic("arm/target_angle");
    final DoublePublisher simAnglePub;
    final DoublePublisher encoderAnglePub;
    final DoublePublisher targetAnglePub;

    public ArmSimulation() {
        m_upperArm = m_root.append(new MechanismLigament2d("upperarm", 4, 0));
        m_armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 212.59, 690, 0.6555486, 0.507867133, 1.781293706, true,
                0.872665);
        SmartDashboard.putData("Arm", m_armMech);

        simAnglePub = Sim_Angle.publish();
        simAnglePub.setDefault(0.0);

        encoderAnglePub = Encoder_Angle.publish();
        encoderAnglePub.setDefault(0.0);

        targetAnglePub = Target_Angle.publish();
        targetAnglePub.setDefault(0.0);

    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_arm, DCMotor.getNEO(1));
    }

    @Override
    public void simulationPeriodic() {
        double motor_volts = m_arm.getAppliedOutput() * m_arm.getBusVoltage();
        m_armSim.setInputVoltage(motor_volts);
        m_armSim.update(0.02);

        get_angle = m_armSim.getAngleRads();
        // System.out.println("Arm Rad per Sec" + m_armSim.getVelocityRadPerSec());
        // System.out.println("motor volts " + m_arm.getBusVoltage() + ", " +
        // m_arm.getAppliedOutput());
        m_angle = get_angle * (180 / Math.PI);
        m_upperArm.setAngle(get_angle * (180 / Math.PI));
        simAnglePub.set(m_upperArm.getAngle());
    }

    public static double getAngle() {
        return get_angle;
    }

    @Override
    public void periodic() {
        super.periodic();
        targetAnglePub.set(getTargetAngle(), NetworkTablesJNI.now());
        encoderAnglePub.set(m_armEncoder.getAbsolutePosition(), NetworkTablesJNI.now());
    }
}
// 688.78 is CG inertia Distance bwetwwn cg and axis is 15.17247438
// length in in 25.809 in
// min 29.5 degrees
// max 82 degrees
// 20lbs
// 9.0718474