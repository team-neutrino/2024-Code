// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorIDs;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_arm = new CANSparkMax(MotorIDs.Arm, MotorType.kBrushless);
  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCONDER);

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults();
  }

  private void setArm(double output) {
    m_arm.setVoltage(output);
  }

  private double armPID(double targetAngle, double currentAngle) {
    double errorSum = 0;
    double lastError = 0;
    double error = targetAngle - currentAngle;
    errorSum += error;
    double change = error - lastError;
    double output = ArmConstants.Arm_kp * error + ArmConstants.Arm_kd * errorSum + ArmConstants.Arm_kd * change;
    return output;
  }

  public void setAngle(double angle) {
    setArm(armPID(angle, m_armEncoder.getAbsolutePosition()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
