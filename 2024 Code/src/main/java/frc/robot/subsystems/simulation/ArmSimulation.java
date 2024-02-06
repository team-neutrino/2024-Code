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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class ArmSimulation extends ArmSubsystem {
    public static double currentSimAngle;
    DutyCycleEncoderSim m_armEncoderSim;
    Mechanism2d m_mech = SubsystemContainer.simOverview.m_mech;
    MechanismRoot2d m_root = m_mech.getRoot("shoulder", 6, 22);
    MechanismLigament2d m_upperArm;
    MechanismRoot2d m_mounterRoot = m_mech.getRoot("mounter", 6, 22);
    MechanismLigament2d m_mounterLigament;
    MechanismRoot2d m_shooterMounterRoot = m_mech.getRoot("shoter_mounter", 6, 22);
    MechanismLigament2d m_shooterMounterLigament;

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

    double kG = 0.001;

    static double simTargetAngle;

    double armMassKg = 7;
    double radius = 0.6555;
    double armMOI = armMassKg * Math.pow(radius, 2) * ((double) 1 / 3);

    public ArmSimulation() {
        m_armEncoderSim = new DutyCycleEncoderSim(m_armEncoder);
        m_upperArm = m_root.append(new MechanismLigament2d("upperarm", 24, 0));
        m_mounterLigament = m_mounterRoot.append(new MechanismLigament2d("mounter", 12, 0));
        m_shooterMounterLigament = m_shooterMounterRoot.append(new MechanismLigament2d("shooterMounter", 14.5, 0));
        IntakeSimulation.m_beambreakLigament = m_upperArm.append(new MechanismLigament2d("BeamBreak", .5, 0));
        Shooter.m_wheel_ligament = m_shooterMounterLigament.append(new MechanismLigament2d("wheel", 2, 0));
        IntakeSimulation.m_indexWheelLigament = m_mounterLigament.append(new MechanismLigament2d("index", 2, 0));
        m_armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 212.59, armMOI, 0.6555486, 0.507867133, 1.781293706, true,
                0.872665);

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

    public static void setSimTargetAngle(double targetAngle) {
        simTargetAngle = targetAngle;
    }

    @Override
    public void simulationPeriodic() {
        currentSimAngle = m_armSim.getAngleRads() * (180 / Math.PI);
        // the cos term * gravity accel * mass = force that is perpendicular to the arm,
        // * center of mass (r / 2) gives the torque
        double gravity_torque_comp = (Math.cos(currentSimAngle * (Math.PI / 180)) * 9.8 * armMassKg) * radius / 2;
        double ff = gravity_torque_comp * kG;
        motor_volts = pidSim.runPid(0.8, 0.0, 0.0,
                ff,
                simTargetAngle, currentSimAngle, 0.0, -13, 13);

        m_armSim.setInputVoltage(motor_volts);
        m_armSim.update(0.02);

        m_armEncoderSim.setAbsolutePosition(currentSimAngle);
        m_upperArm.setAngle(currentSimAngle);
        simAnglePub.set(m_upperArm.getAngle());

        m_mounterLigament.setAngle(currentSimAngle);
        m_shooterMounterLigament.setAngle(currentSimAngle);
    }

    @Override
    public void periodic() {
        super.periodic();
        targetAnglePub.set(simTargetAngle, NetworkTablesJNI.now());
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