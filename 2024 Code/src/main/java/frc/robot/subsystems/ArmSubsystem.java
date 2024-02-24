// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.simulation.PIDChangerSimulation;
import frc.robot.util.ArmEncoderContainer;

public class ArmSubsystem extends SubsystemBase {
  protected CANSparkMax m_arm = new CANSparkMax(MotorIDs.Arm, MotorType.kBrushless);
  protected DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  protected double m_angle;
  protected double m_targetAngle;
  private boolean m_inPosition;
  public int i = 0;
  private SparkPIDController pidController;
  private ArmEncoderContainer armEncoderContainer;

  public final PIDChangerSimulation PIDSimulation = new PIDChangerSimulation(ArmConstants.Arm_kp, ArmConstants.Arm_ki,
      ArmConstants.Arm_kd);

  public ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.FF_ks, ArmConstants.FF_kg, ArmConstants.FF_ks,
      ArmConstants.FF_ka);

  public ArmSubsystem() {
    m_arm.restoreFactoryDefaults();
    m_arm.setIdleMode(IdleMode.kBrake);

    armEncoderContainer = new ArmEncoderContainer(m_arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle));
    armEncoderContainer.setPositionConversionFactor(360);
    armEncoderContainer.setInverted(true);
    armEncoderContainer.setZeroOffset(ArmConstants.ARM_ABS_ENCODER_ZERO_OFFSET);

    m_arm.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15);
    m_arm.setSmartCurrentLimit(Constants.ArmConstants.ARM_CURRENT_LIMIT);

    pidController = m_arm.getPIDController();
    pidController.setP(ArmConstants.Arm_kp, 0);
    pidController.setI(ArmConstants.Arm_ki, 0);
    pidController.setD(ArmConstants.Arm_kd, 0);
    pidController.setFeedbackDevice(armEncoderContainer.m_armEncoder);
    pidController.setPositionPIDWrappingMaxInput(360);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingEnabled(true);

    m_arm.burnFlash();
  }

  public double getTargetAngle() {
    return m_targetAngle;
  }

  /**
   * 
   * @return returns current arm angle in degrees (-180, 180)
   */
  public double getArmAngleDegrees() {
    return adjustAngleOut(armEncoderContainer.getPosition());
  }

  /**
   * 
   * @return returns current arm angle in radians (-pi, pi)
   */
  public double getArmAngleRadians() {
    return adjustAngleOut(armEncoderContainer.getPosition()) * (Math.PI / 180);
  }

  /**
   * Set arm reference angle, target angle should be in degrees. Will be converted
   * to (0, 360) before sent
   * to the motor controller
   * 
   * @param targetAngle must be in degrees (-180, 180)
   */
  public void setArmReferenceAngle(double targetAngle) {

    m_targetAngle = targetAngle;

    if (targetAngle > ArmConstants.ARM_UPPER_LIMIT) {
      targetAngle = ArmConstants.ARM_UPPER_LIMIT;
    } else if (targetAngle < ArmConstants.ARM_LOWER_LIMIT) {
      targetAngle = ArmConstants.ARM_LOWER_LIMIT;
    }

    double feedforward = ArmConstants.FF_kg
        * ((ArmConstants.ARM_CM) * (9.8 * ArmConstants.ARM_MASS_KG * Math.cos(getArmAngleRadians())));

    targetAngle = adjustAngleIn(targetAngle);

    pidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 0, feedforward);
  }

  /**
   * Adjusts the angle going into the pid reference logic to conform to the (0,
   * 360) angle system
   * 
   * @param angle desired angle to adjust
   * @return adjusted angle that is ready to use as a reference
   */
  private double adjustAngleIn(double angle) {
    if (angle < 0) {
      angle += 360;
    }
    return angle;
  }

  /**
   * Adjusts the arm angle going out of the subsystem so that it conforms with
   * (-180, 180)
   * (like when the position is being accessed).
   * 
   * @param angle in (0, 360) (raw return from abs encoder)
   * @return angle in (-180, 180)
   */
  private double adjustAngleOut(double angle) {
    if (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  public double limitShiftAngle(double angle) {
    if (angle > ArmConstants.ARM_UPPER_LIMIT) {
      return ArmConstants.ARM_UPPER_LIMIT;
    } else if (angle < ArmConstants.ARM_LOWER_LIMIT) {
      return ArmConstants.ARM_LOWER_LIMIT;
    }
    return angle;
  }

  private boolean ArmDebouncer() {
    if (Math.abs(getArmAngleDegrees() - m_targetAngle) <= 2) {
      i++;
    } else {
      i = 0;
    }
    return i >= 10;
  }

  public boolean getInPosition() {
    return m_inPosition;
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

  public void setClimb(boolean climb)
  {
    if (climb)
    {
      pidController.setI(0.0001, 0);
    }
  }

  @Override
  public void periodic() {
    m_inPosition = ArmDebouncer();
  }
}
