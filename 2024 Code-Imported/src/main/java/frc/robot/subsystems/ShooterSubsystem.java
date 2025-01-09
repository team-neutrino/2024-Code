package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MessageTimers;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax m_shooterMotor = new SparkMax(MotorIDs.SHOOTER_MOTOR1, MotorType.kBrushless);
  private SparkMax m_followerMotor = new SparkMax(MotorIDs.SHOOTER_MOTOR2, MotorType.kBrushless);
  
  private SparkMaxConfig m_shooterMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig m_shooterFollowerConfig = new SparkMaxConfig();

  private RelativeEncoder m_shooterEncoder;
  private RelativeEncoder m_followerEncoder;
  private SparkClosedLoopController m_pidController;
  private Debouncer m_shootDebouncer;

    private ControlType m_shootControlType;
    private double m_targetVoltage;
    private double m_targetRPM;

    private boolean m_atSpeed;

  public ShooterSubsystem() {
    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_pidController = m_shooterMotor.getClosedLoopController();
    //m_pidController.setFeedbackDevice(m_shooterEncoder);
    m_shooterMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_shooterMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
      m_shooterMotorConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_shooterMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    m_followerEncoder = m_followerMotor.getEncoder();
    m_shooterFollowerConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);
    m_shooterFollowerConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_shooterFollowerConfig.softLimit.forwardSoftLimitEnabled(false);
    m_shooterFollowerConfig.softLimit.reverseSoftLimitEnabled(false);   
    m_shooterFollowerConfig.follow(m_shooterMotor, true);

    m_shooterMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(ShooterConstants.WHEEL_P, ShooterConstants.WHEEL_I, ShooterConstants.WHEEL_D,ShooterConstants.WHEEL_FF,ClosedLoopSlot.kSlot1)
    .iZone(ShooterConstants.WHEEL_IZONE)
    .outputRange(0, 1);

        // shooter motor CAN messages rates
        m_shooterMotor.config.signals.faultsPeriodMs(5);
        m_shooterMotor.config.signals.primaryEncoderVelocityPeriodMs(10);
        m_shooterMotor.config.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);
        m_shooterMotor.config.signals.analogVoltagePeriodMs(MessageTimers.Status3);
        m_shooterMotor.config.signals.externalOrAltEncoderPosition(MessageTimers.Status4);
        m_shooterMotor.config.signals.externalOrAltEncoderVelocity(MessageTimers.Status4);
        // m_shooterMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,
        // MessageTimers.Status5);
        m_shooterMotor.config.signals.motorTemperaturePeriodMs(MessageTimers.Status6);
        m_shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistParameters.kPersistParameters);

        // shooter follower CAN messages rates
        m_followerMotor.config.signals.faultsPeriodMs(MessageTimers.Status0);
        m_followerMotor.config.signals.primaryEncoderVelocityPeriodMs(MessageTimers.Status1);
        m_followerMotor.config.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);
        m_followerMotor.config.signals.analogVoltagePeriodMs(MessageTimers.Status3);
        m_followerMotor.config.signals.externalOrAltEncoderPosition(MessageTimers.Status4);
        m_followerMotor.config.signals.externalOrAltEncoderVelocity(MessageTimers.Status4);
        // m_followerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,
        // MessageTimers.Status5);

    // m_shooterMotor.burnFlash();
    // m_followerMotor.burnFlash();

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
      m_shooterMotorConfig.smartCurrentLimit(Constants.ShooterConstants.HIGH_SHOOTER_CURRENT_LIMIT);
      m_shooterFollowerConfig.smartCurrentLimit(Constants.ShooterConstants.HIGH_SHOOTER_CURRENT_LIMIT);
    } else {
      m_shooterMotorConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
      m_shooterFollowerConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    }

  @Override
  public void periodic() {
    if (m_shootControlType == ControlType.kVelocity) {
      m_pidController.setReference(m_targetRPM, SparkBase.ControlType.kVelocity);
    } else {
      m_shooterMotor.setVoltage(m_targetVoltage);
    }
}