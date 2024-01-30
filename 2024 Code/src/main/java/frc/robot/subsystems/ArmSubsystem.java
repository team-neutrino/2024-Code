// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.simulation.PIDChangerSimulation;

public class ArmSubsystem extends SubsystemBase {
  protected CANSparkMax m_arm = new CANSparkMax(MotorIDs.Arm, MotorType.kBrushless);
  protected DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  private double errorSum;
  private double lastError;
  private double lastAngle;
  private double lastVelocity;
  private double PIDoutput;
  protected double m_angle;
  private double m_targetAngle;
  private boolean m_inPosition;
  public int i = 0;
  private SparkAbsoluteEncoder feedbackSensor;
  private SparkPIDController pidController;
  //SparkAbsoluteEncoder m;

  public final PIDChangerSimulation PIDSimulation = new PIDChangerSimulation(ArmConstants.Arm_kp, ArmConstants.Arm_ki,
      ArmConstants.Arm_kd);

  public ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.FF_ks, ArmConstants.FF_kg, ArmConstants.FF_ks,
      ArmConstants.FF_ka);

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults();

    feedbackSensor = m_arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_arm.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1);

    pidController = m_arm.getPIDController();
    pidController.setFeedbackDevice(feedbackSensor);
  }

  public double getTargetAngle() {
    return m_targetAngle;
  }

  public double getCurrentAngle() {
    return m_angle;
  }

  public double getArmPose() {
    return m_armEncoder.getAbsolutePosition() * 100.0;
  }

  public void armPID(double targetAngle, double feedforward) {

    

    if ((m_angle >= ArmConstants.INTAKE_LIMIT) ||
        (m_angle <= ArmConstants.AMP_LIMIT)) {
      
    } else {
      
    }

    // m_targetAngle = targetAngle;
    // m_targetAngle = withinRange(m_targetAngle);
    // double error = targetAngle - m_angle;
    // errorSum += error;
    // double change = (error - lastError) / .02;
    // lastError = error;
    // PIDoutput = PIDSimulation.GetP() * error + PIDSimulation.GetI() * errorSum
    //     + PIDSimulation.GetD() * change;

    // armChecker(PIDoutput);
  }

  private void setArmVoltage(double desiredVolt) {
    if ((m_angle >= ArmConstants.INTAKE_LIMIT && desiredVolt > 0) ||
        (m_angle <= ArmConstants.AMP_LIMIT && desiredVolt < 0)) {
      m_arm.setVoltage(0);
    } else {
      m_arm.setVoltage(desiredVolt);
    }
  }

  private boolean ArmDebouncer() {
    if (Math.abs(m_angle - m_targetAngle) <= 5) {
      i++;
    } else {
      i = 0;
    }
    return i >= 10;
  }

  public boolean getInPosition() {
    return m_inPosition;
  }

  private double withinRange(double check) {
    if (check >= ArmConstants.INTAKE_LIMIT) {
      return ArmConstants.INTAKE_LIMIT;

    } else if (check <= ArmConstants.AMP_LIMIT) {
      return ArmConstants.AMP_LIMIT;

    } else {
      return check;
    }
  }

  @Override
  public void periodic() {
    m_angle = getArmPose();
    m_inPosition = ArmDebouncer();

  }
}
