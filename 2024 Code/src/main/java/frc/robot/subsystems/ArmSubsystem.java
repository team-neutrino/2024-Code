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
  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  private double errorSum;
  private double lastError;
  private double error;
  private double change;
  private double PIDoutput;

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults();
  }

  private double getArmPose() {
    return m_armEncoder.getAbsolutePosition() * 100.0;
  }

  public double armPID(double targetAngle) {
    double angle = getArmPose();
    error = targetAngle - angle;
    errorSum += error;
    change = error - lastError;
    PIDoutput = ArmConstants.Arm_kp * error + ArmConstants.Arm_kd * errorSum + ArmConstants.Arm_kd * change;
    System.out.println("error " + error + " target " + targetAngle + " actual " + angle + " output " + PIDoutput);
    return PIDoutput;
  }

  public void armChecker(double desiredVolt) {
    if ((getArmPose() >= ArmConstants.INTAKE_LIMIT && desiredVolt > 0) ||
        (getArmPose() <= ArmConstants.AMP_LIMIT && desiredVolt < 0)) {
      m_arm.setVoltage(0);
    } else {
      m_arm.setVoltage(desiredVolt);
    }
  }

  @Override
  public void periodic() {

  }
}
