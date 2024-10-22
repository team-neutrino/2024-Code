// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrapper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;

public class SwerveSim extends SubsystemBase {
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  DoubleTopic Motor_Voltage = nt.getDoubleTopic("swerve/motor_voltage");
  final DoublePublisher motorVoltage_pub;

  /** Creates a new SwerveSim. */
  public SwerveSim() {
    motorVoltage_pub = Motor_Voltage.publish();
    motorVoltage_pub.setDefault(0.0);
  }

  private TalonFX m_talonFX = new TalonFX(4);

  TalonFXSimState m_talonFXSim = m_talonFX.getSimState();
  double fakeMotorVoltage = m_talonFXSim.getMotorVoltage();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
