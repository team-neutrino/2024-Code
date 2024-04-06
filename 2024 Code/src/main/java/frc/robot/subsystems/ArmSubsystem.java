// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import java.util.TreeMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.util.CalculateP;
import frc.robot.Constants.LEDConstants.States;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkFlex m_armMotor = new CANSparkFlex(MotorIDs.Arm, MotorType.kBrushless);
  private AbsoluteEncoder m_armEncoder;
  private double m_targetAngle = 0.0;
  private boolean m_inPosition;
  private Debouncer m_armDebouncer;
  private SparkPIDController m_pidController;
  private int m_PIDslot;
  private double m_error;
  private double m_oldAngle;
  private Timer m_timer;
  TreeMap<Double, Double> m_mapOfP;
  States commandState;

  public ArmSubsystem() {
    m_mapOfP = new TreeMap<Double, Double>();
    m_mapOfP.put(2.0, 0.022);
    m_mapOfP.put(7.0, 0.04);
    initializeMotorControllers();
    m_armDebouncer = new Debouncer(ArmConstants.DEBOUNCE_TIME, DebounceType.kRising);
    m_targetAngle = Constants.ArmConstants.INTAKE_POSE;
    m_timer = new Timer();
  }

  public void defaultArm() {
    setArmReferenceAngle(ArmConstants.INTAKE_POSE);
  }

  // converts to (0, 360)
  private double adjustAngleIn(double angle) {
    if (angle < 0) {
      angle += 360;
    }
    return angle;
  }

  // converts to (0, 180)
  private double adjustAngleOut(double angle) {
    if (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  // -180, 180
  public double getArmAngleDegrees() {
    return adjustAngleOut(m_armEncoder.getPosition());
  }

  // -pi, pi
  public double getArmAngleRadians() {
    return adjustAngleOut(m_armEncoder.getPosition()) * (Math.PI / 180);
  }

  public double getTargetAngle() {
    return m_targetAngle;
  }

  public boolean getInPosition() {
    return m_inPosition;
  }

  public double limitArmAngle(double angle) {
    if (angle > ArmConstants.ARM_UPPER_LIMIT) {
      return ArmConstants.ARM_UPPER_LIMIT;
    } else if (angle < ArmConstants.ARM_LOWER_LIMIT) {
      return ArmConstants.ARM_LOWER_LIMIT;
    } else if (Double.isNaN(angle)) {
      return ArmConstants.INTAKE_POSE;
    }
    return angle;
  }

  public void initializeMotorControllers() {
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake);

    m_armEncoder = m_armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_armEncoder.setPositionConversionFactor(360);
    m_armEncoder.setZeroOffset(ArmConstants.ARM_ABS_ENCODER_ZERO_OFFSET);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 100);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 20);
    m_armMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);
    m_armMotor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);

    m_pidController = m_armMotor.getPIDController();
    m_pidController.setP(ArmConstants.Arm_kp, 0);
    m_pidController.setI(ArmConstants.Arm_ki, 0);
    m_pidController.setD(ArmConstants.Arm_kd, 0);
    m_pidController.setFeedbackDevice(m_armEncoder);
    m_pidController.setPositionPIDWrappingMaxInput(360);
    m_pidController.setPositionPIDWrappingMinInput(0);
    m_pidController.setPositionPIDWrappingEnabled(true);

    // climb settings
    m_pidController.setP(ArmConstants.ClimbArm_ki, 1);
    m_pidController.setI(ArmConstants.ClimbArm_kp, 1);
    m_pidController.setD(ArmConstants.ClimbArm_kd, 1);
    m_pidController.setIZone(ArmConstants.ClimbIZone, 1);

    m_pidController.setP(ArmConstants.FastArm_kp, 2);
    m_pidController.setI(ArmConstants.Arm_ki, 2);
    m_pidController.setD(ArmConstants.Arm_kd, 2);

    m_armMotor.burnFlash();
  }

  public void setArmReferenceAngle(double targetAngle) {
    m_targetAngle = targetAngle;
    pidChanger();
  }

  public void setClimbReferenceAngle() {
    m_targetAngle = ArmConstants.CLIMB_ANGLE;
    m_PIDslot = 1;
  }

  // in degrees. converted to (0, 360)
  private void updateArmAngle(double targetAngle, int PIDslot) {

    targetAngle = limitArmAngle(targetAngle);
    targetAngle = adjustAngleIn(targetAngle);
    m_pidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, PIDslot, feedForwardCalculation());
  }

  private double feedForwardCalculation() {
    double currentAngle = getArmAngleRadians();
    double filtAngle = 0.98 * currentAngle + 0.02 * m_oldAngle;
    m_oldAngle = filtAngle;

    return ArmConstants.FF_kg
        * ((ArmConstants.ARM_CM) * (9.8 * ArmConstants.ARM_MASS_KG * Math.cos(filtAngle)));
  }

  public void commandStart() {
    m_timer.restart();
  }

  public States getCommandState() {
    return commandState;
  }

  public void setCommandState(States state) {
    commandState = state;
  }

  private void pidChanger() {
    if (m_timer.get() < Constants.ArmConstants.timeBeforeSwitchPID) {
      m_PIDslot = 2;
    } else {
      m_PIDslot = 0;
    }
  }

  public boolean aboveAngle(double angleThreshold) {
    return (getArmAngleDegrees() > angleThreshold);
  }

  @Override
  public void periodic() {
    m_error = Math.abs(getArmAngleDegrees() - m_targetAngle);
    updateArmAngle(m_targetAngle, m_PIDslot);
    m_inPosition = m_armDebouncer
        .calculate(Math.abs(getArmAngleDegrees() - m_targetAngle) <= ArmConstants.POSITION_ERROR_THRESHOLD);
  }
}
