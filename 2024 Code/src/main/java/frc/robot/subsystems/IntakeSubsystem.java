package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.Constants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {

    private double indexVoltage = 0.0;
    private double intakeVoltage = 0.0;
    private boolean m_indexBeam = false;
    private boolean m_intakeBeam = false;
    private boolean m_centered = false;
    private int i;

    protected RelativeEncoder m_intakeEncoder;
    protected RelativeEncoder m_intakeEncoder2;
    protected RelativeEncoder m_indexEncoder;
    protected RelativeEncoder m_indexEncoder2;

    protected CANSparkMax m_intakeMotor = new CANSparkMax(MotorIDs.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    protected CANSparkMax m_intakeMotor2 = new CANSparkMax(MotorIDs.INTAKE_MOTOR_TWO,
            CANSparkLowLevel.MotorType.kBrushless);
    protected CANSparkMax m_indexMotor = new CANSparkMax(MotorIDs.INDEX_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    protected CANSparkMax m_indexMotor2 = new CANSparkMax(MotorIDs.INDEX_MOTOR2, CANSparkLowLevel.MotorType.kBrushless);

    protected DigitalInput m_intakeBeamBreak = new DigitalInput(DigitalConstants.INTAKE_MOTOR_BEAMBREAK);
    protected DigitalInput m_indexBeamBreak = new DigitalInput(DigitalConstants.INDEX_MOTOR_BEAMBREAK);

    SlewRateLimiter intakeLimiter = new SlewRateLimiter(5.0);

    public IntakeSubsystem() {
        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeEncoder2 = m_intakeMotor2.getEncoder();
        m_indexEncoder = m_indexMotor.getEncoder();
        m_indexEncoder2 = m_indexMotor2.getEncoder();

        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor2.restoreFactoryDefaults();
        m_intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT);
        m_intakeMotor2.setSmartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT);
        m_intakeMotor2.follow(m_intakeMotor, false);

        m_indexMotor.restoreFactoryDefaults();
        m_indexMotor.setSmartCurrentLimit(Constants.IntakeConstants.INDEX_CURRENT_LIMIT);

        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        m_intakeMotor2.setIdleMode(IdleMode.kCoast);

        m_indexMotor2.restoreFactoryDefaults();
        m_indexMotor2.setSmartCurrentLimit(Constants.IntakeConstants.INDEX_CURRENT_LIMIT);
        m_indexMotor2.follow(m_indexMotor, true);

        m_intakeMotor.burnFlash();
        m_intakeMotor2.burnFlash();
        m_indexMotor.burnFlash();
        m_indexMotor2.burnFlash();
    }

    public void runIntake() {
        if (isBeamBrokenIntake()) {
            stopIntake();
        } else {

            intakeVoltage = IntakeConstants.INTAKE_MOTOR_VOLTAGE;
        }
    }

    private boolean IndexFeedCheck() {
        double threshold = 20;
        if (centerNote()) {
            i++;
        } else {
            i = 0;
        }

        m_centered = i > threshold;
        return m_centered;

    }

    public boolean isCentered() {
        return m_centered;
    }

    public void runIndexFeed() {
        IndexFeedCheck();
        if (noNote()) {
            indexVoltage = IntakeConstants.INDEX_MOTOR_VOLTAGE_INTAKE;
        } else if (tooFarNote()) {
            indexVoltage = -IntakeConstants.INDEX_MOTOR_VOLTAGE_POSITION;
        } else if (isCentered()) {
            stopIndex();
        }

    }

    public void intakeNote() {
        runIndexFeed();
        runIntake();
    }

    public void runIndexShoot() {
        indexVoltage = IntakeConstants.INDEX_MOTOR_VOLTAGE_SHOOT;
    }

    public void runIndex() {
        indexVoltage = IntakeConstants.INDEX_MOTOR_VOLTAGE_INTAKE;
    }

    public void runIntakeReverse() {
        intakeVoltage = -IntakeConstants.INTAKE_MOTOR_VOLTAGE;
    }

    public void runIndexReverse() {
        indexVoltage = -IntakeConstants.INDEX_MOTOR_VOLTAGE_INTAKE;

    }

    public void runIndexJitterReverse() {
        indexVoltage = -IntakeConstants.INDEX_JITTER_MOTOR_VOLTAGE;

    }

    public void runIndexJitter() {
        indexVoltage = IntakeConstants.INDEX_JITTER_MOTOR_VOLTAGE;

    }

    public void stopIntake() {
        intakeVoltage = 0;
    }

    public void stopIndex() {
        indexVoltage = 0;
    }

    public double getIntakeVelocity() {
        return m_intakeEncoder.getVelocity();
    }

    public double getIndexVelocity() {
        return m_indexEncoder.getVelocity();
    }

    public double getIndexVoltage() {
        return indexVoltage;
    }

    public void resetEncoders() {
        m_intakeEncoder.setPosition(0);
        m_intakeEncoder2.setPosition(0);
        m_indexEncoder.setPosition(0);
        m_indexEncoder2.setPosition(0);
    }

    public void indexJitter() {
        if (isBeamBrokenIndex()) {
            runIndexJitterReverse();
        } else {
            runIndexJitter();
        }
    }

    /**
     * Gets the current state of the beam break: true means the beam break is
     * tripped
     * 
     * @return The state of the intake beam break.
     */
    public boolean isBeamBrokenIntake() {
        return m_intakeBeam;
    }

    public boolean isBeamBrokenIndex() {
        return m_indexBeam;
    }

    public boolean noNote() {
        return !m_indexBeam && !m_intakeBeam;
    }

    public boolean hasNote() {
        return m_indexBeam || m_intakeBeam;
    }

    public boolean tooFarNote() {
        return m_indexBeam && m_intakeBeam;
    }

    public boolean centerNote() {
        return m_intakeBeam && !m_indexBeam;
    }

    @Override
    public void periodic() {
        m_indexMotor.set(indexVoltage);
        m_intakeMotor.set(intakeLimiter.calculate(intakeVoltage));
        m_indexBeam = !m_indexBeamBreak.get();
        m_intakeBeam = !m_intakeBeamBreak.get();
    }
}