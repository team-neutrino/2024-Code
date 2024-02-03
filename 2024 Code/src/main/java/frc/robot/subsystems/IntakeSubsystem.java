package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.Constants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {

    protected RelativeEncoder m_intakeEncoder;
    protected RelativeEncoder m_indexEncoder;

    protected CANSparkMax m_intakeMotor = new CANSparkMax(MotorIDs.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    protected CANSparkMax m_indexMotor = new CANSparkMax(MotorIDs.INDEX_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

    protected DigitalInput m_intakeBeamBreak = new DigitalInput(DigitalConstants.INTAKE_MOTOR_BEAMBREAK);

    public IntakeSubsystem() {
        m_intakeMotor.restoreFactoryDefaults();
        m_indexMotor.restoreFactoryDefaults();
        m_indexMotor.setSmartCurrentLimit(Constants.IntakeConstants.INDEX_CURRENT_LIMIT);

    }

    public void runIntake() {
        m_intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void runIndex() {
        m_indexMotor.set(IntakeConstants.INDEX_MOTOR_SPEED);
    }

    public void runIntakeReverse() {
        m_intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void runIndexReverse() {
        m_indexMotor.set(-IntakeConstants.INDEX_MOTOR_SPEED);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public void stopIndex() {
        m_indexMotor.stopMotor();
    }

    public double getIntakeVelocity() {
        return m_intakeEncoder.getVelocity();
    }

    public double getIndexVelocity() {
        return m_indexEncoder.getVelocity();
    }

    public void resetEncoders() {
        m_intakeEncoder.setPosition(0);
        m_indexEncoder.setPosition(0);
    }

    /**
     * Gets the current state of the beam break: false means the beam break is
     * tripped
     * 
     * @return The state of the intake beam break.
     */
    public boolean getBeamBreak() {
        return m_intakeBeamBreak.get();
    }

    public void indexApprove(boolean allow) {
        if (allow) {
            runIndex();
        } else {
            stopIndex();
        }
    }

}
