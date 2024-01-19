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
  private double errorSum;
  private double lastError;
  private double error;
  private double change;
  private double PIDoutput;

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults();
  }

  private double armPID(double targetAngle, double currentAngle) {
    error = targetAngle - currentAngle;
    errorSum += error;
    change = error - lastError;
    PIDoutput = ArmConstants.Arm_kp * error + ArmConstants.Arm_kd * errorSum + ArmConstants.Arm_kd * change;
    return PIDoutput;
  }

  public void setAngle(double angle) {
    m_arm.setVoltage(armPID(angle, m_armEncoder.getAbsolutePosition()));
  }

  public void armChecker(double desiredVolt) {
    if ((m_armEncoder.getAbsolutePosition() >= 180 && desiredVolt > ArmConstants.INTAKE_LIMIT) ||
        (m_armEncoder.getAbsolutePosition() <= ArmConstants.AMP_LIMIT && desiredVolt < 0)) {
      m_arm.setVoltage(0);
    } else {
      m_arm.setVoltage(desiredVolt);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
