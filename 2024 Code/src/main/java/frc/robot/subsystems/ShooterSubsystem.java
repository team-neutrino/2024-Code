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

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_shooter = new CANSparkMax(MotorIDs.shooter, MotorType.kBrushless);
  // initialize encoder
  private RelativeEncoder m_shooterEncoder;
  // private JoystickButton m_bumperRight
  private SparkPIDController m_pidController;
  private DigitalInput m_beamBreak = new DigitalInput(DigitalConstants.SHOOTER_BEAMBREAK);
  private double WHEEL_P = 0.25;
  private double WHEEL_I = 0.0006;
  private double WHEEL_D = 0;
  private double WHEEL_FF = 0.0975;
  private double m_targetRPM;

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
    System.out.println(m_beamBreak.get());
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
    m_pidController.setReference(m_targetRPM, CANSparkBase.ControlType.kVelocity);
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

  public void periodic() {
  }
}
