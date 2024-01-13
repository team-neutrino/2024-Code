package frc.robot;

import com.revrobotics.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private int angleID;
    private int speedID;
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkAnalogSensor absAngleEncoder;
    private RelativeEncoder speedEncoder;
    private SparkPIDController anglePID;
    private SparkPIDController speedPID;

    public SwerveModule(int speedID, int angleID) {
        this.angleID = angleID;
        this.speedID = speedID;
        angleMotor = new CANSparkMax(angleID, CANSparkLowLevel.MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedID, CANSparkLowLevel.MotorType.kBrushless);

        angleMotor.restoreFactoryDefaults();
        speedMotor.restoreFactoryDefaults();

        if (speedID == 1) {
            speedMotor.setInverted(true);
        }

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
        }

        else {
            out = 360 - out;
        }
        return Rotation2d.fromDegrees(out);
    }

    private double adjustAngleOut() {
        double out = 0;
        if (angleID == 2) {
            if (SwerveConstants.FRA_OFFSET < 180) {
                out = smallAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.FRA_OFFSET);
            } else {
                out = largeAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.FRA_OFFSET);
            }
        } else if (angleID == 4) {
            if (SwerveConstants.FLA_OFFSET < 180) {
                out = smallAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.FLA_OFFSET);
            } else {
                out = largeAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.FLA_OFFSET);
            }
        } else if (angleID == 6) {
            if (SwerveConstants.BRA_OFFSET < 180) {
                out = smallAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.BRA_OFFSET);
            } else {
                out = largeAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.BRA_OFFSET);
            }
        } else if (angleID == 8) {
            if (SwerveConstants.BLA_OFFSET < 180) {
                out = smallAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.BLA_OFFSET);
            } else {
                out = largeAngleAdjusterOut(absAngleEncoder.getPosition(), SwerveConstants.BLA_OFFSET);
            }
        }

        return out;
    }

    private double largeAngleAdjusterOut(double angle, double constant) {
        if (angle > (360 - constant)) {
            angle -= (360 - constant);
        } else {
            angle += constant;
        }
        return angle;
    }

    private double smallAngleAdjusterOut(double angle, double constant) {
        if (angle < (360 - constant)) {
            angle += constant;
        } else {
            angle += (constant - 360);
        }
        return angle;
    }

    private double adjustAngleIn(double angle) {
        if (angleID == 2) {
            if (SwerveConstants.FRA_OFFSET < 180) {
                angle = smallAngleAdjusterIn(angle, SwerveConstants.FRA_OFFSET);
            } else {
                angle = largeAngleAdjusterIn(angle, SwerveConstants.FRA_OFFSET);
            }
        }

        else if (angleID == 4) {
            if (SwerveConstants.FLA_OFFSET < 180) {
                angle = smallAngleAdjusterIn(angle, SwerveConstants.FLA_OFFSET);
            } else {
                angle = largeAngleAdjusterIn(angle, SwerveConstants.FLA_OFFSET);
            }
        }

        else if (angleID == 6) {
            if (SwerveConstants.BRA_OFFSET < 180) {
                angle = smallAngleAdjusterIn(angle, SwerveConstants.BRA_OFFSET);
            } else {
                angle = largeAngleAdjusterIn(angle, SwerveConstants.BRA_OFFSET);
            }
        }

        else if (angleID == 8) {
            if (SwerveConstants.BLA_OFFSET < 180) {
                angle = smallAngleAdjusterIn(angle, SwerveConstants.BLA_OFFSET);
            } else {
                angle = largeAngleAdjusterIn(angle, SwerveConstants.BLA_OFFSET);
            }
        }

        return angle;
    }

    private double largeAngleAdjusterIn(double angle, double constant) {
        if (angle < constant) {
            return angle + (360 - constant);
        } else {
            return angle - constant;
        }
    }

    private double smallAngleAdjusterIn(double angle, double constant) {
        if (angle > constant) {
            angle -= constant;
        } else {
            angle += (-constant + 360);
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
        return absAngleEncoder.getPosition();
    }

    public double getVoltage() {
        return absAngleEncoder.getVoltage();
    }
}
