// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.wpilibj.Timer;
import java.util.TreeMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MessageTimers;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.LEDConstants.States;

public class ArmSubsystem extends SubsystemBase {
  private static final int m_sparkHandle = MotorIDs.Arm; //maybe change spark handle
  private SparkFlex m_armMotor = new SparkFlex(MotorIDs.Arm, SparkLowLevel.MotorType.kBrushless);
  private SparkMaxConfig m_armMotorConfig = new SparkMaxConfig();
  private SparkMaxConfigAccessor m_armMotorConfigAccessor = new SparkMaxConfigAccessor(m_sparkHandle);
  private AbsoluteEncoder m_armEncoder;
  private double m_targetAngle = 0.0;
  private boolean m_inPosition;
  private Debouncer m_armDebouncer;
  private SparkClosedLoopController m_pidController;
  private ClosedLoopSlot m_PIDslot;
  private double m_error;
  private double m_oldAngle;
  private Timer m_timer;
  private int m_armWrapCounter;
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
   m_armMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_armMotor.configure(m_armMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //change?
    m_armEncoder = m_armMotor.getAbsoluteEncoder();
   m_armMotorConfig.encoder.positionConversionFactor(360);
   m_armMotorConfig.absoluteEncoder.zeroOffset(ArmConstants.ARM_ABS_ENCODER_ZERO_OFFSET);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, MessageTimers.Status0);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, MessageTimers.Status1);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, MessageTimers.Status2);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, MessageTimers.Status3);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus4, MessageTimers.Status4);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus5, 17);
    // m_armMotor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus6, MessageTimers.Status6);
   m_armMotorConfig.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);

    m_armMotor.configure(m_armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   m_armMotorConfig.smartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);

   m_armMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ArmConstants.ClimbArm_kp, ArmConstants.ClimbArm_ki, ArmConstants.ClimbArm_kd,ClosedLoopSlot.kSlot1)
    .pid(ArmConstants.FastArm_kp, ArmConstants.Arm_ki, ArmConstants.Arm_kd,ClosedLoopSlot.kSlot2)
    .iZone(ArmConstants.ClimbIZone)
    .positionWrappingMaxInput(360)
    .positionWrappingMinInput(0)
    .positionWrappingEnabled(true);
    m_pidController = m_armMotor.getClosedLoopController();
  }

  public void setArmReferenceAngle(double targetAngle) {
    m_targetAngle = targetAngle;
    pidChanger();
  }

  public void setClimbReferenceAngle() {
    m_targetAngle = ArmConstants.CLIMB_ANGLE;
    m_PIDslot = ClosedLoopSlot.kSlot1;
  }

  // in degrees. converted to (0, 360)
  private void updateArmAngle(double targetAngle, ClosedLoopSlot PIDslot) {

    targetAngle = limitArmAngle(targetAngle);
    targetAngle = adjustAngleIn(targetAngle);
    m_pidController.setReference(targetAngle, SparkBase.ControlType.kPosition, m_PIDslot, feedForwardCalculation());
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
      m_PIDslot = ClosedLoopSlot.kSlot2;
    } else {
      m_PIDslot = ClosedLoopSlot.kSlot0;
    }
  }

  public boolean aboveAngle(double angleThreshold) {
    return (getArmAngleDegrees() > angleThreshold);
  }

  public void keepArmWrapped() {
    m_armWrapCounter++;
    if (m_armWrapCounter >= 50) {
      if (!m_armMotorConfigAccessor.closedLoop.getPositionWrappingEnabled()) {
       m_armMotorConfig.closedLoop.positionWrappingEnabled(true);
      } else {
        m_armWrapCounter = 0;
      }
    }
  }

  @Override
  public void periodic() {
    m_error = Math.abs(getArmAngleDegrees() - m_targetAngle);
    updateArmAngle(m_targetAngle, m_PIDslot);
    keepArmWrapped();
    m_inPosition = m_armDebouncer
        .calculate(Math.abs(getArmAngleDegrees() - m_targetAngle) <= ArmConstants.POSITION_ERROR_THRESHOLD);
  }
}
