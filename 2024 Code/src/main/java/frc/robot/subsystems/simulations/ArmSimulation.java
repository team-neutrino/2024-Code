// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.ArmSubsystem;

/** Add your docs here. */
public class ArmSimulation extends ArmSubsystem {
    Mechanism2d m_armMech = new Mechanism2d(10, 10);
    MechanismRoot2d m_root = m_armMech.getRoot("shoulder", 3, 3);
    MechanismLigament2d m_upperArm;

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoder_Angle = nt.getDoubleTopic("Arm Angle");

    ArmSimulation() {
        m_upperArm = m_root.append(new MechanismLigament2d("upperarm", 4, 0));

    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_arm, DCMotor.getNEO(1));
    }
}
