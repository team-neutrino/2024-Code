package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MessageTimers;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {

    private double indexVoltage = 0.0;
    private double intakeVoltage = 0.0;
    private boolean m_indexBeam = false;
    private boolean m_intakeBeam = false;
    private boolean m_noteReady = false;

    private RelativeEncoder m_intakeEncoder;
    private RelativeEncoder m_indexEncoder;
    private SparkMax m_intakeMotor = new SparkMax(MotorIDs.INTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_intakeFollower = new SparkMax(MotorIDs.INTAKE_MOTOR_TWO, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_indexMotor = new SparkMax(MotorIDs.INDEX_MOTOR, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_indexFollower = new SparkMax(MotorIDs.INDEX_MOTOR2, SparkLowLevel.MotorType.kBrushless);

    private SparkMaxConfig m_intakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_intakeFollowerConfig = new SparkMaxConfig();
    private SparkMaxConfig m_indexMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_indexFollowerConfig = new SparkMaxConfig();

    private DigitalInput m_intakeBeamBreak = new DigitalInput(DigitalConstants.INTAKE_MOTOR_BEAMBREAK);
    private DigitalInput m_indexBeamBreak = new DigitalInput(DigitalConstants.INDEX_MOTOR_BEAMBREAK);

    private SlewRateLimiter intakeLimiter = new SlewRateLimiter(IntakeConstants.INTAKE_SLEW_RATE);

    private Debouncer m_intakeDebouncer;

    public IntakeSubsystem() {
        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_indexEncoder = m_indexMotor.getEncoder();

        m_intakeMotor.configure(m_intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeFollower.configure(m_intakeFollowerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_indexMotor.configure(m_indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_indexFollower.configure(m_indexFollowerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_intakeMotorConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        // potentially check later if it applys for both motors
        m_intakeFollowerConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        m_intakeFollowerConfig.follow(m_intakeMotor, false);

        m_indexMotorConfig.smartCurrentLimit(IntakeConstants.INDEX_CURRENT_LIMIT);

        m_intakeMotorConfig.idleMode(IdleMode.kCoast);
        m_intakeFollowerConfig.idleMode(IdleMode.kCoast);

        m_indexFollowerConfig.smartCurrentLimit(IntakeConstants.INDEX_CURRENT_LIMIT);
        m_indexFollowerConfig.follow(m_indexMotor, true);

        // intake motor CAN messages rates
        m_intakeMotorConfig.signals.faultsPeriodMs(10);
        m_intakeMotorConfig.signals.primaryEncoderVelocityPeriodMs(MessageTimers.Status1);
        m_intakeMotorConfig.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);
        m_intakeMotorConfig.signals.analogVoltagePeriodMs(MessageTimers.Status3);
        m_intakeMotorConfig.signals.externalOrAltEncoderPosition(MessageTimers.Status4);
        m_intakeMotorConfig.signals.externalOrAltEncoderVelocity(MessageTimers.Status4);
        // m_intakeMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,
        // MessageTimers.Status5);
        m_intakeMotorConfig.signals.motorTemperaturePeriodMs(MessageTimers.Status6);
        m_intakeMotor.configure(m_intakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // intake follower CAN messages rates
        m_intakeFollowerConfig.signals.faultsPeriodMs(MessageTimers.Status0);
        m_intakeFollowerConfig.signals.primaryEncoderVelocityPeriodMs(MessageTimers.Status1);
        m_intakeFollowerConfig.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);
        m_intakeFollowerConfig.signals.analogVoltagePeriodMs(MessageTimers.Status3);
        m_intakeFollowerConfig.signals.externalOrAltEncoderPosition(MessageTimers.Status4);
        m_intakeFollowerConfig.signals.externalOrAltEncoderVelocity(MessageTimers.Status4);
        // m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,
        // MessageTimers.Status5);
        m_intakeFollowerConfig.signals.motorTemperaturePeriodMs(MessageTimers.Status6);
        m_intakeFollower.configure(m_indexFollowerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // index motor CAN messages rates
        m_indexMotorConfig.signals.faultsPeriodMs(10);
        m_indexMotorConfig.signals.primaryEncoderVelocityPeriodMs(MessageTimers.Status1);
        m_indexMotorConfig.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);
        m_indexMotorConfig.signals.analogVoltagePeriodMs(MessageTimers.Status3);
        m_indexMotorConfig.signals.externalOrAltEncoderPosition(MessageTimers.Status4);
        m_indexMotorConfig.signals.externalOrAltEncoderVelocity(MessageTimers.Status4);
        // m_indexMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,
        // MessageTimers.Status5);
        m_indexMotorConfig.signals.motorTemperaturePeriodMs(MessageTimers.Status6);
        m_indexMotor.configure(m_indexMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // index follower CAN messages rates
        m_indexFollowerConfig.signals.faultsPeriodMs(MessageTimers.Status0);
        m_indexFollowerConfig.signals.primaryEncoderVelocityPeriodMs(MessageTimers.Status1);
        m_indexFollowerConfig.signals.primaryEncoderPositionPeriodMs(MessageTimers.Status2);
        m_indexFollowerConfig.signals.analogVoltagePeriodMs(MessageTimers.Status3);
        m_indexFollowerConfig.signals.externalOrAltEncoderPosition(MessageTimers.Status4);
        m_indexFollowerConfig.signals.externalOrAltEncoderVelocity(MessageTimers.Status4);
        // m_intakeFollower.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,
        // MessageTimers.Status5);
        m_indexFollowerConfig.signals.motorTemperaturePeriodMs(MessageTimers.Status6);
        m_indexFollower.configure(m_indexFollowerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_intakeDebouncer = new Debouncer(IntakeConstants.INTAKE_ERROR_THRESHOLD, DebounceType.kRising);
    }

    public void defaultIntake() {
        stopIntake();
        stopIndex();
    }

    public double getIndexVoltage() {
        return indexVoltage;
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