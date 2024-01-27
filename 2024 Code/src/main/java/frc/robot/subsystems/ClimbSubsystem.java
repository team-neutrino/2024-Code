// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

/**
 * NOTES:
 * 1/26 it is still unclear how locking the mechanism in place during teleop
 * will work.
 * 
 * TODO: soft upper (use encoder revs?) and lower limits (limit switch)
 */
public class ClimbSubsystem extends SubsystemBase {
    /*
     * Motor controllers
     */
    private CANSparkMax m_climb1 = new CANSparkMax(Constants.MotorIDs.CLIMB_MOTOR1,
            MotorType.kBrushless);
    private CANSparkMax m_climb2 = new CANSparkMax(Constants.MotorIDs.CLIMB_MOTOR2, MotorType.kBrushless);

    /**
     * Relative encorders, initialized in the constructor with the 
     * helper method "initializeMotor"
     */
    private RelativeEncoder m_climbEncoder1;
    private RelativeEncoder m_climbEncoder2;

    /**
     * Public constructor to be invoked in RobotContainer,
     * instantiates encoders.
     */
    public ClimbSubsystem() {
        m_climbEncoder1 = initializeMotor(m_climb1, false);
        m_climbEncoder2 = initializeMotor(m_climb2, false);
        // makes the second climb motor turn the same direction with the same
        // power as the first one
        m_climb2.follow(m_climb1);
    }

    /**
     * Helper method that returns an instance of the given motor controller's
     * relative encoder and sets the motor controller's default behaviours.
     * 
     * @param motorController The motor controller to configure.
     * @param inverted Inversion state of the encoder, true means inverted.
     * @return An instance of the given motor controller's encoder.
     */
    private RelativeEncoder initializeMotor(CANSparkMax p_motorController, boolean inverted) {

        p_motorController.restoreFactoryDefaults();
        p_motorController.setIdleMode(IdleMode.kBrake);
        p_motorController.setInverted(inverted);

        // p_motorController.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // p_motorController.enableSoftLimit(SoftLimitDirection.kForward, true);
        // p_motorController.setSmartCurrentLimit();
        // p_motorController.setSoftLimit(SoftLimitDirection.kForward, 2000);

        // m_climb1.setOpenLoopRampRate(4); used in 2022 climb - not needed here?

        // .setPositionConversionFactor(DriverConstants.ENCODER_POSITION_CONVERSION);
        // .setVelocityConversionFactor(DriverConstants.ENCODER_VELOCITY_CONVERSION);
        // ^^ May be needed in the future, keep these comments pls

        return p_motorController.getEncoder();
    }

    /**
     * Starts the arm motors to the constant determined extend speed.
     * {@link frc.robot.Constants.ClimbConstants#CLIMB_EXTEND_MOTOR_SPEED}
     * 
     * The {@link com.revrobotics.CANSparkBase#follow(CANSparkBase leader)}
     * method was used on m_climb2 to connect the motors, meaning that method
     * calls changing the state of m_climb1 equally change the state of m_climb2,
     * removing the need to call both the motors.
     * 
     * NOTE: CURRENT CONSTANT IS A PLACEHOLDER VALUE
     */
    public void extendClimberArms() {
        m_climb1.set(Constants.ClimbConstants.CLIMB_EXTEND_MOTOR_SPEED);
    }

    /**
     * Stops the arm motors.
     * 
     * The {@link com.revrobotics.CANSparkBase#follow(CANSparkBase)}
     * method was used on m_climb2 to connect the motors, meaning that method
     * calls changing the state of m_climb1 equally change the state of m_climb2,
     * removing the need to call both the motors.
     */
    public void stopClimberArms() {
        m_climb1.stopMotor();
    }

    /**
     * Starts the arm motors to the constant speed
     * {@link frc.robot.Constants.ClimbConstants#CLIMB_RETRACT_MOTOR_SPEED}
     * 
     * The {@link com.revrobotics.CANSparkBase#follow(CANSparkBase)}
     * method was used on m_climb2 to connect the motors, meaning that method
     * calls changing the state of m_climb1 equally change the state of m_climb2,
     * removing the need to call both the motors.
     * 
     * NOTE: current constant is the same as the extend climber,
     * just negative - should there be a separate value?
     */
    public void rectractClimberArms() {
        m_climb1.set(Constants.ClimbConstants.CLIMB_RETRACT_MOTOR_SPEED);
    }

    /**
     * Resets all encoders
     */
    public void resetEncoders() {
        m_climbEncoder1.setPosition(0);
        m_climbEncoder2.setPosition(0);
    }

    /**
     * Getter for the arm motors' positions.
     * 
     * @return An array of the arm motors' positions, index 0 being
     *         {@link #m_climbEncoder1} and index 1 being {@link #m_climbEncoder2}.
     */
    public double[] getArmEncoderPosition() {
        return new double[] { m_climbEncoder1.getPosition(), m_climbEncoder2.getPosition() };
    }

    /**
     * Getter for the arm motors' velocities.
     * 
     * @return An array of the arm motors' velocities, index 0 being
     *         {@link #m_climbEncoder1} and index 1 being {@link #m_climbEncoder2}.
     */

    public double[] getArmEncoderVelocity() {
        return new double[] { m_climbEncoder1.getVelocity(), m_climbEncoder2.getVelocity() };
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
