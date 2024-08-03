package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MessageTimers;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_shooterMotor = new CANSparkMax(MotorIDs.SHOOTER_MOTOR1, MotorType.kBrushless);
  private CANSparkMax m_followerMotor = new CANSparkMax(MotorIDs.SHOOTER_MOTOR2, MotorType.kBrushless);
  private RelativeEncoder m_shooterEncoder;
  private RelativeEncoder m_followerEncoder;
  private SparkPIDController m_pidController;
  private Debouncer m_shootDebouncer;

  private ControlType m_shootControlType;
  private double m_targetVoltage;
  private double m_targetRPM;

  private boolean m_atSpeed;

  public ShooterSubsystem() {
    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_pidController = m_shooterMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_shooterEncoder);
    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_shooterMotor.setInverted(false);
    m_shooterMotor.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_shooterMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    m_followerEncoder = m_followerMotor.getEncoder();
    m_followerMotor.setIdleMode(IdleMode.kCoast);
    m_followerMotor.setInverted(true);
    m_followerMotor.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_followerMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
    m_followerMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false);
    m_followerMotor.follow(m_shooterMotor, true);

    m_pidController.setP(ShooterConstants.WHEEL_P);
    m_pidController.setI(ShooterConstants.WHEEL_I);
    m_pidController.setD(ShooterConstants.WHEEL_D);
    m_pidController.setFF(ShooterConstants.WHEEL_FF);
    m_pidController.setIZone(ShooterConstants.WHEEL_IZONE);
    m_pidController.setOutputRange(0, 1);

    // shooter motor CAN messages rates
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 5);
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10);
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, MessageTimers.Status2);
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, MessageTimers.Status3);
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, MessageTimers.Status4);
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, MessageTimers.Status5);
    m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, MessageTimers.Status6);

    // shooter follower CAN messages rates
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, MessageTimers.Status0);
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, MessageTimers.Status1);
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, MessageTimers.Status2);
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, MessageTimers.Status3);
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, MessageTimers.Status4);
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, MessageTimers.Status5);
    m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, MessageTimers.Status6);

    m_shooterMotor.burnFlash();
    m_followerMotor.burnFlash();

    m_shootDebouncer = new Debouncer(ShooterConstants.DEBOUNCE_TIME, DebounceType.kRising);
  }

  public void defaultShooter() {
    setVoltage(SubsystemContainer.intakeSubsystem.hasNote() ? ShooterSpeeds.INITIAL_SHOOTER_SPEED : 0.0);
  }

  public boolean approveShoot() {
    return m_atSpeed;
  }

  public boolean aboveRPM(double p_rpm) {
    return (getShooterRPM() > p_rpm);
  }

  public double getShooterRPM() {
    return m_shooterEncoder.getVelocity();
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public void setTargetRPM(double p_targetRPM) {
    m_targetRPM = p_targetRPM;
    m_shootControlType = ControlType.kVelocity;
  }

  public void setVoltage(double voltage) {
    m_targetVoltage = voltage;
    m_shootControlType = ControlType.kVoltage;
  }

  public void useHighCurrentLimits(boolean isHighCurrent) {
    if (isHighCurrent) {
      m_shooterMotor.setSmartCurrentLimit(Constants.ShooterConstants.HIGH_SHOOTER_CURRENT_LIMIT);
      m_followerMotor.setSmartCurrentLimit(Constants.ShooterConstants.HIGH_SHOOTER_CURRENT_LIMIT);
    } else {
      m_shooterMotor.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
      m_followerMotor.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    }
  }

  @Override
  public void periodic() {
    if (m_shootControlType == ControlType.kVelocity) {
      m_pidController.setReference(m_targetRPM, CANSparkBase.ControlType.kVelocity);
    } else {
      m_shooterMotor.setVoltage(m_targetVoltage);
    }

    m_atSpeed = m_shootDebouncer
        .calculate(Math.abs(getTargetRPM() - getShooterRPM()) <= ShooterConstants.RPM_ERROR_THRESHOLD);
  }
}