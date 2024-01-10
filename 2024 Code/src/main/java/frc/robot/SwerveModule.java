package frc.robot;
import com.revrobotics.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {

    private int angleID;
    private int speedID;
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkAnalogSensor absAngleEncoder;
    private RelativeEncoder speedEncoder;
    private SparkPIDController anglePID;
    private SparkPIDController speedPID;

    public SwerveModule(int speedID, int angleID){
        this.angleID = angleID;
        this.speedID = speedID;
        angleMotor = new CANSparkMax(angleID, CANSparkLowLevel.MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedID, CANSparkLowLevel.MotorType.kBrushless);
        angleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        speedMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        absAngleEncoder = angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        speedEncoder = speedMotor.getEncoder();
        absAngleEncoder.setInverted(true);
        absAngleEncoder.setPositionConversionFactor(360/3.3);
        speedEncoder.setPositionConversionFactor(Constants.DimensionConstants.WHEEL_CIRCUMFERENCE/Constants.SwerveConstants.GEAR_RATIO);
        speedEncoder.setVelocityConversionFactor(Constants.DimensionConstants.WHEEL_CIRCUMFERENCE/(60*Constants.SwerveConstants.GEAR_RATIO));
        anglePID = angleMotor.getPIDController();
        speedPID = speedMotor.getPIDController();
        anglePID.setFeedbackDevice(absAngleEncoder);
        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(360);
        anglePID.setPositionPIDWrappingMinInput(0);
        anglePID.setP(Constants.SwerveConstants.ANGLE_P, 0);
        speedPID.setP(Constants.SwerveConstants.SPEED_P, 0);
    }

    // public Rotation2d getOptimizationAngle(){
    //     return;
    // }

     
}
