package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {

    private double indexVoltage = 0.0;
    private double intakeVoltage = 0.0;
    private boolean m_indexBeam = false;
    private boolean m_intakeBeam = false;
    private boolean m_noteReady = false;

    private RelativeEncoder m_intakeEncoder;
    private RelativeEncoder m_indexEncoder;

    private CANSparkMax m_intakeMotor = new CANSparkMax(MotorIDs.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_intakeFollower = new CANSparkMax(MotorIDs.INTAKE_MOTOR_TWO,
            CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_indexMotor = new CANSparkMax(MotorIDs.INDEX_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_indexFollower = new CANSparkMax(MotorIDs.INDEX_MOTOR2, CANSparkLowLevel.MotorType.kBrushless);

    private DigitalInput m_intakeBeamBreak = new DigitalInput(DigitalConstants.INTAKE_MOTOR_BEAMBREAK);
    private DigitalInput m_indexBeamBreak = new DigitalInput(DigitalConstants.INDEX_MOTOR_BEAMBREAK);

    private SlewRateLimiter intakeLimiter = new SlewRateLimiter(IntakeConstants.INTAKE_SLEW_RATE);

    private Debouncer m_intakeDebouncer;

    public IntakeSubsystem() {
        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_indexEncoder = m_indexMotor.getEncoder();

        m_intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        m_intakeFollower.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        m_intakeFollower.follow(m_intakeMotor, false);

        m_indexMotor.setSmartCurrentLimit(IntakeConstants.INDEX_CURRENT_LIMIT);

        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        m_intakeFollower.setIdleMode(IdleMode.kCoast);

        m_indexFollower.setSmartCurrentLimit(IntakeConstants.INDEX_CURRENT_LIMIT);
        m_indexFollower.follow(m_indexMotor, true);

        // intake motor CAN messages rates
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
        m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);

        // intake follower CAN messages rates
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 100);
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
        m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);

        // index motor CAN messages rates
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
        m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);

        // index follower CAN messages rates
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 100);
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 500);
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 500);
        m_indexFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 500);

        m_intakeMotor.burnFlash();
        m_intakeFollower.burnFlash();
        m_indexMotor.burnFlash();
        m_indexFollower.burnFlash();

        m_intakeDebouncer = new Debouncer(IntakeConstants.INTAKE_ERROR_THRESHOLD, DebounceType.kRising);
    }

    public void defaultIntake() {
        stopIntake();
        stopIndex();
    }

    public double getIndexVelocity() {
        return m_indexEncoder.getVelocity();
    }

    public double getIndexVoltage() {
        return indexVoltage;
    }

    public double getIntakeVelocity() {
        return m_intakeEncoder.getVelocity();
    }

    public boolean hasNoNote() {
        return !m_indexBeam && !m_intakeBeam;
    }

    public boolean hasNote() {
        return m_indexBeam || m_intakeBeam;
    }

    public boolean isBeamBrokenIntake() {
        return m_intakeBeam;
    }

    public boolean isBeamBrokenIndex() {
        return m_indexBeam;
    }

    public boolean isNoteReady() {
        return m_noteReady;
    }

    private boolean isNoteCentered() {
        return m_intakeBeam && !m_indexBeam;
    }

    public boolean isNoteTooFar() {
        return m_indexBeam && m_intakeBeam;
    }

    public void runIndexFeed() {
        if (hasNoNote()) {
            indexVoltage = IntakeConstants.INDEX_MOTOR_VOLTAGE_INTAKE;
        } else if (isNoteTooFar()) {
            indexVoltage = -IntakeConstants.INDEX_MOTOR_VOLTAGE_POSITION;
        } else if (isNoteReady()) {
            stopIndex();
        }

    }

    public void runIndexReverse() {
        indexVoltage = -IntakeConstants.INDEX_MOTOR_VOLTAGE_INTAKE;

    }

    public void runIndexShoot() {
        indexVoltage = IntakeConstants.INDEX_MOTOR_VOLTAGE_SHOOT;
    }

    private void runIntake() {
        if (isBeamBrokenIntake()) {
            stopIntake();
        } else {

            intakeVoltage = IntakeConstants.INTAKE_MOTOR_VOLTAGE;
        }
    }

    public void runIntakeReverse() {
        intakeVoltage = -IntakeConstants.INTAKE_MOTOR_VOLTAGE;
    }

    public void smartIntake() {
        runIndexFeed();
        runIntake();
    }

    public void stopIndex() {
        indexVoltage = 0;
    }

    public void stopIntake() {
        intakeVoltage = 0;
    }

    @Override
    public void periodic() {
        m_indexMotor.set(indexVoltage);
        m_intakeMotor.set(intakeLimiter.calculate(intakeVoltage));
        m_indexBeam = !m_indexBeamBreak.get();
        m_intakeBeam = !m_intakeBeamBreak.get();

        m_noteReady = m_intakeDebouncer.calculate(isNoteCentered());
    }

}