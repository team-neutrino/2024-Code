// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDs;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkFlex m_arm = new CANSparkFlex(MotorIDs.Arm, MotorType.kBrushless);
  private AbsoluteEncoder m_armEncoder;
  private double m_targetAngle = 0.0;
  private boolean m_inPosition;
  private Debouncer m_armDebouncer;
  private SparkPIDController pidController;

  public ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.FF_ks, ArmConstants.FF_kg, ArmConstants.FF_ks,
      ArmConstants.FF_ka);

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults();
    m_arm.setIdleMode(IdleMode.kBrake);

    m_armEncoder = m_arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_arm.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15);
    m_arm.setSmartCurrentLimit(Constants.ArmConstants.ARM_CURRENT_LIMIT);

    pidController = m_arm.getPIDController();
    pidController.setP(ArmConstants.Arm_kp, 0);
    pidController.setI(ArmConstants.Arm_ki, 0);
    pidController.setD(ArmConstants.Arm_kd, 0);
    pidController.setFeedbackDevice(m_armEncoder);
    pidController.setPositionPIDWrappingMaxInput(360);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingEnabled(true);

    // climb settings
    pidController.setP(ArmConstants.Arm_kp, 1);
    pidController.setI(0.0001, 1);
    pidController.setIZone(30, 1);

    m_arm.burnFlash();

    m_armDebouncer = new Debouncer(ArmConstants.DEBOUNCE_TIME, DebounceType.kRising);
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

  public boolean ArmDebouncer() {
    return m_armDebouncer
        .calculate(Math.abs(getArmAngleDegrees() - m_targetAngle) <= ArmConstants.POSITION_ERROR_THRESHOLD);
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

  public double limitShiftAngle(double angle) {
    if (angle > ArmConstants.ARM_UPPER_LIMIT) {
      return ArmConstants.ARM_UPPER_LIMIT;
    } else if (angle < ArmConstants.ARM_LOWER_LIMIT) {
      return ArmConstants.ARM_LOWER_LIMIT;
    }
    return angle;
  }

  public void resetEverything() {
    m_arm.restoreFactoryDefaults();
    m_arm.setIdleMode(IdleMode.kBrake);

    m_armEncoder = m_arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_arm.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15);
    m_arm.setSmartCurrentLimit(Constants.ArmConstants.ARM_CURRENT_LIMIT);

    pidController = m_arm.getPIDController();
    pidController.setP(ArmConstants.Arm_kp, 0);
    pidController.setI(ArmConstants.Arm_ki, 0);
    pidController.setD(ArmConstants.Arm_kd, 0);
    pidController.setFeedbackDevice(m_armEncoder);
    pidController.setPositionPIDWrappingMaxInput(360);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingEnabled(true);

    // climb settings
    pidController.setP(ArmConstants.Arm_kp, 1);
    pidController.setI(0.0001, 1);
    pidController.setIZone(30, 1);

    m_arm.burnFlash();
  }

  // in degrees. converted to (0, 360)
  public void setArmReferenceAngle(double targetAngle) {

    if (targetAngle > ArmConstants.ARM_UPPER_LIMIT) {
      targetAngle = ArmConstants.ARM_UPPER_LIMIT;
    } else if (targetAngle < ArmConstants.ARM_LOWER_LIMIT) {
      targetAngle = ArmConstants.ARM_LOWER_LIMIT;
    } else if (Double.isNaN(targetAngle)) {
      return;
    }

    m_targetAngle = targetAngle;

    double feedforward = ArmConstants.FF_kg
        * ((ArmConstants.ARM_CM) * (9.8 * ArmConstants.ARM_MASS_KG * Math.cos(getArmAngleRadians())));

    targetAngle = adjustAngleIn(targetAngle);

    pidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 0, feedforward);
  }

  public void setClimbReferenceAngle() {
    double targetAngle = adjustAngleIn(-20);

    double feedforward = ArmConstants.FF_kg
        * ((ArmConstants.ARM_CM) * (9.8 * ArmConstants.ARM_MASS_KG * Math.cos(getArmAngleRadians())));

    pidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 1, feedforward);
  }

  public double withinRange(double check) {
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
