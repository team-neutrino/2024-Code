package frc.robot;

import com.revrobotics.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    public static class MotorCfg {
        public MotorCfg(int can_id, boolean inverted) {
            m_can_id = can_id;
            m_inverted = inverted;
            m_angle_offset = 0.0;
        }

        public MotorCfg(int can_id, boolean inverted, double angle_offset) {
            m_can_id = can_id;
            m_inverted = inverted;
            m_angle_offset = angle_offset;
        }

        public int CanId() {
            return m_can_id;
        }

        public boolean IsInverted() {
            return m_inverted;
        }

        public double AngleOffset() {
            return m_angle_offset;
        }

        protected int m_can_id;
        protected boolean m_inverted;
        protected double m_angle_offset;

    }

    private MotorCfg angle_motor_cfg;
    private MotorCfg speed_motor_cfg;
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkAnalogSensor absAngleEncoder;
    private RelativeEncoder speedEncoder;
    private SparkPIDController anglePID;
    private SparkPIDController speedPID;

    public SwerveModule(MotorCfg speed_motor_configuration, MotorCfg angle_motor_configuration) {
        speed_motor_cfg = speed_motor_configuration;
        angle_motor_cfg = angle_motor_configuration;
        angleMotor = new CANSparkMax(angle_motor_cfg.CanId(), CANSparkLowLevel.MotorType.kBrushless);
        speedMotor = new CANSparkMax(speed_motor_cfg.CanId(), CANSparkLowLevel.MotorType.kBrushless);

        angleMotor.restoreFactoryDefaults();
        speedMotor.restoreFactoryDefaults();

        speedMotor.setInverted(speed_motor_cfg.IsInverted());
        angleMotor.setInverted(angle_motor_cfg.IsInverted());

        angleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        speedMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        absAngleEncoder = angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        speedEncoder = speedMotor.getEncoder();
        absAngleEncoder.setInverted(true);
        absAngleEncoder.setPositionConversionFactor(360 / 3.3);
        speedEncoder.setPositionConversionFactor(
                Constants.DimensionConstants.WHEEL_CIRCUMFERENCE / Constants.SwerveConstants.GEAR_RATIO);
        speedEncoder.setVelocityConversionFactor(
                Constants.DimensionConstants.WHEEL_CIRCUMFERENCE / (60 * Constants.SwerveConstants.GEAR_RATIO));
        anglePID = angleMotor.getPIDController();
        speedPID = speedMotor.getPIDController();
        anglePID.setFeedbackDevice(absAngleEncoder);
        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(360);
        anglePID.setPositionPIDWrappingMinInput(0);
        anglePID.setP(Constants.SwerveConstants.ANGLE_P, 0);
        speedPID.setP(Constants.SwerveConstants.SPEED_P, 0);
    }

    public Rotation2d getOptimizationAngle() {
        double out = adjustAngleOut();
        if (out <= 0) {
            out *= -1;
        } else {
            out = 360 - out;
        }

        return Rotation2d.fromDegrees(out);
    }

    private double adjustAngleOut() {
        final double OFFSET = angle_motor_cfg.AngleOffset();
        double angle = absAngleEncoder.getPosition();

        if (angle > OFFSET) {
            angle -= OFFSET;
        } else {
            angle += -OFFSET + 360;
        }

        return angle;
    }

    private double adjustAngleIn(double angle) {
        final double OFFSET = angle_motor_cfg.AngleOffset();

        if (angle < 360 - OFFSET) {
            angle += OFFSET;
        } else {
            angle += OFFSET - 360;
        }

        return angle;
    }

    public void setAnglePID(double reference) {
        reference = adjustAngleIn(reference);
        anglePID.setReference(reference, CANSparkBase.ControlType.kPosition, 0);
    }

    public void setSpeedPID(double reference, double feedforward) {
        speedPID.setReference(reference, CANSparkBase.ControlType.kVelocity, 0, feedforward);
    }

    public double getAbsoluteAngle() {
        return adjustAngleOut();
    }

    public double getRawAbsoluteAngle() {
        return absAngleEncoder.getPosition();
    }

    public double getVoltage() {
        return absAngleEncoder.getVoltage();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(speedEncoder.getPosition(), Rotation2d.fromDegrees(adjustAngleOut()));
    }
}
