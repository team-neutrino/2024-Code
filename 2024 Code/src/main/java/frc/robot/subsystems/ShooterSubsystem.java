package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.MotorConstants;

public class ShooterSubsystem extends SubsystemBase {
   private CANSparkMax m_shooter =
      new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);
      private RelativeEncoder m_shooterEncoder;
      private JoystickButton m_bumperRight;
    

  public ShooterSubsystem(JoystickButton p_bumperRight){
    m_bumperRight = p_bumperRight;
    m_shooter.restoreFactoryDefaults();
    m_shooter.setIdleMode(IdleMode.kBrake);
  }
  public double getshooterEncoder() {
    return m_shooterEncoder.getPosition();
  }

  public double getshooterVel() {
    return m_shooterEncoder.getVelocity();
  }
  public void resetEncoders(RelativeEncoder encoder){
    encoder.setPosition(0);
  }
  public void setVoltage(double voltage) {
    m_shooter.setVoltage(voltage);
  }

  public void getVoltage(double voltage){
    m_shooter.setVoltage(voltage);
  }

  public void periodic() {
    
   }
}
