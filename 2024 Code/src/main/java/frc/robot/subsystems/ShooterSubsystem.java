package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ShooterSubsystem extends SubsystemBase {
   private CANSparkMax m_shooter =
      new CANSparkMax(MotorConstants.shooter, MotorType.kBrushless);
      private RelativeEncoder m_shooterEncoder;

  public ShooterSubsystem(){
    m_shooter.restoreFactoryDefaults();
    m_shooter.setIdleMode(IdleMode.kBrake);
  }
  public double shooterEncoder() {
    return m_shooterEncoder.getPosition();
  }

  public double getR1Vel() {
    return m_shooterEncoder.getVelocity();
  }
  public void resetEncoders(RelativeEncoder encoder){
    encoder.setPosition(0);
  }
  public void periodic() {
    
   }
}
