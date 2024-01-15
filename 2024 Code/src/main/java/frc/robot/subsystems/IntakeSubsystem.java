package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {

    private RelativeEncoder m_intakeEncoder;

    private CANSparkMax m_intakeMotor = new CANSparkMax(MotorIDs.INTAKE_MOTOR, MotorType.kBrushless);

    public IntakeSubsystem() {
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeMotor.setSmartCurrentLimit(30, 35); // from previous code idk what it does
    }

    public void runIntake() {
        m_intakeMotor.set(MotorIDs.INTAKE_MOTOR_SPEED);
    }

    public void runIntakeReverse() {
        m_intakeMotor.set(-MotorIDs.INTAKE_MOTOR_SPEED);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public double getIntakeEncoder() {
        return m_intakeEncoder.getVelocity();
    }

    public void resetEncoders() {
        m_intakeEncoder.setPosition(0);
    }

}
