package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.simulation.PIDChangerSimulationShooter;

public class ShooterSubsystem extends SubsystemBase {
  protected CANSparkMax m_shooterMotor = new CANSparkMax(MotorIDs.SHOOTER_MOTOR1, MotorType.kBrushless);
  protected CANSparkMax m_followerMotor = new CANSparkMax(MotorIDs.SHOOTER_MOTOR2, MotorType.kBrushless);
  protected RelativeEncoder m_shooterEncoder;
  protected RelativeEncoder m_followerEncoder;
  private SparkPIDController m_pidController;
  protected double WHEEL_P = 0.00075;
  protected double WHEEL_I = 0.000001;
  protected double WHEEL_D = 0;
  protected double WHEEL_FF = 0.00021;
  protected double WHEEL_IZONE = 250;
  protected double m_targetRPM;
  private int counter;
  final private double APPROVE_ERROR_THRESHOLD = 200;
  final private double APPROVE_COUNTER_THRESHOLD = 5;
  final private double COUNTER_ERROR_THRESHOLD = 200;
  private boolean approve = false;

  public final PIDChangerSimulationShooter PIDSimulationShooter = new PIDChangerSimulationShooter(WHEEL_P, WHEEL_I,
      WHEEL_D, WHEEL_FF, approve);

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

    m_pidController.setP(WHEEL_P);
    m_pidController.setI(WHEEL_I);
    m_pidController.setD(WHEEL_D);
    m_pidController.setFF(WHEEL_FF);
    m_pidController.setIZone(WHEEL_IZONE);
    m_pidController.setOutputRange(0, 1);

    m_shooterMotor.burnFlash();
    m_followerMotor.burnFlash();
  }

  public double getShooterRPM() {
    return m_shooterEncoder.getVelocity();
  }

  public void setVoltage(double voltage) {
    m_shooterMotor.setVoltage(voltage);
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public void setTargetRPM(double p_targetRpm) {
    m_targetRPM = p_targetRpm;
    approvePIDChanges();
    m_pidController.setReference(m_targetRPM, CANSparkBase.ControlType.kVelocity);
  }

  public boolean approveShoot() {
    countCounter();
    return (Math.abs(getShooterRPM() - getTargetRPM()) <= APPROVE_ERROR_THRESHOLD
        && (counter > APPROVE_COUNTER_THRESHOLD));
  }

  private void countCounter() {
    if (Math.abs(getTargetRPM() - getShooterRPM()) < COUNTER_ERROR_THRESHOLD) {
      counter++;
    } else {
      counter = 0;
    }
  }

  public void approvePIDChanges() {
    if (PIDSimulationShooter.simPIDChangeApprove()) {
      WHEEL_P = PIDSimulationShooter.GetP();
      WHEEL_I = PIDSimulationShooter.GetI();
      WHEEL_D = PIDSimulationShooter.GetD();
      WHEEL_FF = PIDSimulationShooter.GetFF();
      m_pidController.setP(WHEEL_P);
      m_pidController.setI(WHEEL_I);
      m_pidController.setD(WHEEL_D);
      m_pidController.setFF(WHEEL_FF);
    }

  }

  @Override
  public void periodic() {
  }
}