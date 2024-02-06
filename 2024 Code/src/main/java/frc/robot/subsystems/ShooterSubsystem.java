package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.subsystems.simulation.PIDChangerSimulationShooter;

public class ShooterSubsystem extends SubsystemBase {
  protected CANSparkMax m_shooter = new CANSparkMax(MotorIDs.SHOOTER_MOTOR, MotorType.kBrushless);
  protected RelativeEncoder m_shooterEncoder;
  private SparkPIDController m_pidController;
  private DigitalInput m_beamBreak = new DigitalInput(DigitalConstants.SHOOTER_BEAMBREAK);
  protected double WHEEL_P = 0.51;
  protected double WHEEL_I = 0.0002;
  protected double WHEEL_D = 0;
  protected double WHEEL_FF = 0.000155;
  protected double m_targetRPM;
  protected double m_rpm_izone = 0.0;
  private int counter;
  final private double APPROVE_ERROR_THRESHOLD = 7;
  final private double APPROVE_COUNTER_THRESHOLD = 9;
  final private double COUNTER_ERROR_THRESHOLD = 10;
  private boolean approve = false;
  // this is a change and a test

  public final PIDChangerSimulationShooter PIDSimulationShooter = new PIDChangerSimulationShooter(WHEEL_P, WHEEL_I,
      WHEEL_D, WHEEL_FF, approve);

  public ShooterSubsystem() {
    m_shooterEncoder = m_shooter.getEncoder();
    m_pidController = m_shooter.getPIDController();
    m_pidController.setFeedbackDevice(m_shooterEncoder);
    m_shooter.restoreFactoryDefaults();
    m_shooter.setIdleMode(IdleMode.kCoast);

    m_pidController.setP(WHEEL_P);
    m_pidController.setI(WHEEL_I);
    m_pidController.setD(WHEEL_D);
    m_pidController.setFF(WHEEL_FF);
    m_pidController.setIZone(500);
    m_pidController.setOutputRange(0, 1);
  }

  public boolean detectedGamePiece() {
    return !m_beamBreak.get();
  }

  public double getshooterEncoder() {
    return m_shooterEncoder.getPosition();
  }

  public double getshooterRpm() {
    return m_shooterEncoder.getVelocity();
  }

  public void resetEncoders(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  public void setVoltage(double voltage) {
    m_shooter.setVoltage(voltage);
  }

  public void getVoltage(double voltage) {
    m_shooter.setVoltage(voltage);
  }

  public double getP() {
    return m_pidController.getP() * 1000.0;
  }

  public double getTargetRPM() {
    return m_targetRPM;

  }

  public void setTargetRPM(double p_targetRpm) {
    m_targetRPM = p_targetRpm;
    approvePIDChanges();
    m_pidController.setReference(m_targetRPM, CANSparkBase.ControlType.kVelocity);

  }

  public void stopShooter() {
    setVoltage(0);
    m_targetRPM = 0;
  }
  
  public boolean approveShoot() {
    countCounter();
    return (Math.abs(getshooterRpm() - getTargetRPM()) <= APPROVE_ERROR_THRESHOLD)
        && (counter > APPROVE_COUNTER_THRESHOLD);
  }

  public double getFF() {
    return m_pidController.getFF() * 1000.0;
  }

  public void setP(double P) {
    m_pidController.setP(P / 1000.0);
  }

  public double getI() {
    return m_pidController.getI() * 1000.0;
  }

  public void setI(double I) {
    m_pidController.setI(I / 1000.0);
  }

  public double getD() {
    return m_pidController.getD() * 1000.0;
  }

  public void setD(double D) {
    m_pidController.setD(D / 1000.0);
  }

  public void setFF(double FF) {
    m_pidController.setFF(FF / 1000.0);
  }

  public boolean magicShooter(double RPM, double TRPM) {
    return Math.abs(RPM - TRPM) <= 10;
  }

  private void countCounter() {
    if (Math.abs(getTargetRPM() - getshooterRpm()) < COUNTER_ERROR_THRESHOLD) {
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
      System.out.println("PID values updated");
    }

  }

  @Override
  public void periodic() {
  }

}