package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterSpeeds;
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
    m_shooterMotor.restoreFactoryDefaults();
    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_shooterMotor.setInverted(false);
    m_shooterMotor.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_shooterMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    m_followerEncoder = m_followerMotor.getEncoder();
    m_followerMotor.restoreFactoryDefaults();
    m_followerMotor.setIdleMode(IdleMode.kCoast);
    m_followerMotor.setInverted(true);
    m_followerMotor.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_followerMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    m_followerMotor.follow(m_shooterMotor, true);

    m_pidController.setP(ShooterConstants.WHEEL_P);
    m_pidController.setI(ShooterConstants.WHEEL_I);
    m_pidController.setD(ShooterConstants.WHEEL_D);
    m_pidController.setFF(ShooterConstants.WHEEL_FF);
    m_pidController.setIZone(ShooterConstants.WHEEL_IZONE);
    m_pidController.setOutputRange(0, 1);

    m_shooterMotor.burnFlash();
    m_followerMotor.burnFlash();

    m_shootDebouncer = new Debouncer(ShooterConstants.DEBOUNCE_TIME, DebounceType.kRising);
  }

  public void defaultShooter() {
    m_targetVoltage = ShooterSpeeds.INITIAL_SHOOTER_SPEED;
  }

  public boolean approveShoot() {
    return m_atSpeed;
  }

  public boolean aboveRPM(double p_rpm) {
    return (getShooterRPM() > p_rpm);
  }

  public double getFollowerRPM() {
    return m_followerEncoder.getVelocity();
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