// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

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
    Mechanism2d m_armMech = new Mechanism2d(10, 10);
    MechanismRoot2d m_root = m_armMech.getRoot("shoulder", 3, 3);
    MechanismLigament2d m_upperArm;
    SingleJointedArmSim m_armSim;
    double m_last_position_radian;
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoder_Angle = nt.getDoubleTopic("Arm Angle");
    final DoublePublisher armAnglesim;
    final DoublePublisher armAngle;

    public ArmSimulation() {
        m_upperArm = m_root.append(new MechanismLigament2d("upperarm", 4, 0));
        m_armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 392, 690, 0.6555486, 0.5148721, 1.43117, true, 0.872665);
        SmartDashboard.putData("Arm", m_armMech);

        armAnglesim = encoder_Angle.publish();
        armAnglesim.setDefault(0.0);

        armAngle = encoder_Angle.publish();
        armAngle.setDefault(0.0);

    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_arm, DCMotor.getNEO(1));
    }

    @Override
    public void simulationPeriodic() {
        double motor_volts = m_arm.getAppliedOutput() * m_arm.getBusVoltage();
        m_armSim.setInputVoltage(motor_volts);
        m_armSim.update(0.02);

        double get_angle = m_armSim.getAngleRads();
        System.out.println("Arm Rad per Sec" + m_armSim.getVelocityRadPerSec());
        m_upperArm.setAngle(get_angle * (180 / Math.PI));
    }

    @Override
    public void periodic() {
        super.periodic();
        armAngle.set(m_armEncoder.getAbsolutePosition(), NetworkTablesJNI.now());
    }
}
// 688.78 is CG inertia Distance bwetwwn cg and axis is 15.17247438
// length in in 25.809 in
// min 29.5 degrees
// max 82 degrees
// 20lbs
// 9.0718474