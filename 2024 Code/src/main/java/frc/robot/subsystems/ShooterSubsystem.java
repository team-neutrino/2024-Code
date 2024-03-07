package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.simulation.PIDChangerSimulationShooter;
import frc.robot.util.SubsystemContainer;

public class ShooterSubsystem extends SubsystemBase {
  protected CANSparkMax m_shooter1 = new CANSparkMax(MotorIDs.SHOOTER_MOTOR1, MotorType.kBrushless);
  protected CANSparkMax m_shooter2 = new CANSparkMax(MotorIDs.SHOOTER_MOTOR2, MotorType.kBrushless);
  protected RelativeEncoder m_shooterEncoder1;
  protected RelativeEncoder m_shooterEncoder2;
  private SparkPIDController m_pidController1;
  private DigitalInput m_beamBreak = new DigitalInput(DigitalConstants.SHOOTER_BEAMBREAK);
  protected double WHEEL_P = 0.00075;
  protected double WHEEL_I = 0.000001;
  protected double WHEEL_D = 0;
  protected double WHEEL_FF = 0.00021;
  protected double m_targetRPM;
  protected double Izone = 250;
  private int counter;
  final private double APPROVE_ERROR_THRESHOLD = 200;
  final private double APPROVE_COUNTER_THRESHOLD = 5;
  final private double COUNTER_ERROR_THRESHOLD = 200;
  private boolean approve = false;

  public final PIDChangerSimulationShooter PIDSimulationShooter = new PIDChangerSimulationShooter(WHEEL_P, WHEEL_I,
      WHEEL_D, WHEEL_FF, approve);

  public ShooterSubsystem() {
    m_shooterEncoder1 = m_shooter1.getEncoder();
    m_pidController1 = m_shooter1.getPIDController();
    m_pidController1.setFeedbackDevice(m_shooterEncoder1);
    m_shooter1.restoreFactoryDefaults();
    m_shooter1.setIdleMode(IdleMode.kCoast);
    m_shooter1.setInverted(false);
    m_shooter1.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_shooter1.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    m_shooterEncoder2 = m_shooter2.getEncoder();
    m_shooter2.restoreFactoryDefaults();
    m_shooter2.setIdleMode(IdleMode.kCoast);
    m_shooter2.setInverted(true);
    m_shooter2.setSmartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
    m_shooter2.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    m_shooter2.follow(m_shooter1, true);

    m_pidController1.setP(WHEEL_P);
    m_pidController1.setI(WHEEL_I);
    m_pidController1.setD(WHEEL_D);
    m_pidController1.setFF(WHEEL_FF);
    m_pidController1.setIZone(Izone);
    m_pidController1.setOutputRange(0, 1);

    m_shooter1.burnFlash();
    m_shooter2.burnFlash();
  }

  public boolean detectedGamePiece() {
    return !m_beamBreak.get();
  }

  public double getShooterEncoder1() {
    return m_shooterEncoder1.getPosition();
  }

  public double getShooterRpm1() {
    return m_shooterEncoder1.getVelocity();
  }

  public void resetEncoders() {
    m_shooterEncoder1.setPosition(0);
    m_shooterEncoder2.setPosition(0);
  }

  public void setVoltage(double voltage) {
    m_shooter1.setVoltage(voltage);
  }

  public double getP() {
    return m_pidController1.getP() * 1000.0;
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public void setTargetRPM(double p_targetRpm) {
    m_targetRPM = p_targetRpm;
    approvePIDChanges();
    m_pidController1.setReference(m_targetRPM, CANSparkBase.ControlType.kVelocity);
  }

  public boolean approveShoot() {
    countCounter();
    return (Math.abs(getShooterRpm1() - getTargetRPM()) <= APPROVE_ERROR_THRESHOLD
        && (counter > APPROVE_COUNTER_THRESHOLD));
  }

  public double getFF() {
    return m_pidController1.getFF() * 1000.0;
  }

  public void setP(double P) {
    m_pidController1.setP(P / 1000.0);
  }

  public double getI() {
    return m_pidController1.getI() * 1000.0;
  }

  public void setI(double I) {
    m_pidController1.setI(I / 1000.0);
  }

  public double getD() {
    return m_pidController1.getD() * 1000.0;
  }

  public void setD(double D) {
    m_pidController1.setD(D / 1000.0);
  }

  public void setFF(double FF) {
    m_pidController1.setFF(FF / 1000.0);
  }

  private void countCounter() {
    if (Math.abs(getTargetRPM() - getShooterRpm1()) < COUNTER_ERROR_THRESHOLD) {
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
      m_pidController1.setP(WHEEL_P);
      m_pidController1.setI(WHEEL_I);
      m_pidController1.setD(WHEEL_D);
      m_pidController1.setFF(WHEEL_FF);
      System.out.println("PID values updated");
    }

  }

  @Override
  public void periodic() {
  }
}