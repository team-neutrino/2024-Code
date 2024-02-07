// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.simulation.ArmSimulation;
import frc.robot.subsystems.simulation.PIDChangerSimulation;

public class ArmSubsystem extends SubsystemBase {
  protected CANSparkMax m_arm = new CANSparkMax(MotorIDs.Arm, MotorType.kBrushless);
  protected DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  protected double m_angle;
  protected double m_targetAngle;
  private boolean m_inPosition;
  public int i = 0;
  private SparkAbsoluteEncoder absEncoder;
  private SparkPIDController pidController;

  public final PIDChangerSimulation PIDSimulation = new PIDChangerSimulation(ArmConstants.Arm_kp, ArmConstants.Arm_ki,
      ArmConstants.Arm_kd);

  public ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.FF_ks, ArmConstants.FF_kg, ArmConstants.FF_ks,
      ArmConstants.FF_ka);

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults(); 

    absEncoder = m_arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    absEncoder.setPositionConversionFactor(360);
    absEncoder.setZeroOffset(ArmConstants.ARM_ABS_ENCODER_ZERO_OFFSET);

    m_arm.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 1);

    pidController = m_arm.getPIDController();
    pidController.setFeedbackDevice(absEncoder);
  }

  public double getTargetAngle() {
    return m_targetAngle;
  }

  public double getArmAngleDegrees()
  {
    if(RobotBase.isSimulation())
    {
      return ArmSimulation.currentSimAngle;
    }
    else
    {
      return absEncoder.getPosition();
    }
  }

  public double getArmAngleRadians()
  {
    return absEncoder.getPosition() * (Math.PI / 180);
  }

  /**
   * Set arm reference angle, target angle should be in degrees
   * 
   * @param targetAngle must be in degrees
   */
  public void setArmReferenceAngle(double targetAngle) {

    m_targetAngle = targetAngle;

    if(targetAngle > ArmConstants.INTAKE_LIMIT)
    {
      targetAngle = ArmConstants.INTAKE_LIMIT;
    } 
    else if(targetAngle < ArmConstants.AMP_LIMIT)
    {
      targetAngle = ArmConstants.AMP_LIMIT;
    }

    double feedforward = ArmConstants.FF_kg * ((ArmConstants.ARM_RADIUS / 2) * (9.8 * ArmConstants.ARM_MASS_KG * Math.cos(getArmAngleRadians())));

    pidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 0, feedforward);
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
    m_inPosition = ArmDebouncer();
  }
}
