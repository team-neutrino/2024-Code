// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import Constants;

import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase

/**
 * TODO:
 * As of 1/11 the climb system is known as follows: some kind of winch mechanism
 * utilizing
 * one motor and encoder to pull the robot up. File to be edited when more
 * details are known.
 * 
 * NOTES:
 * Climber motors controller id's (the first parameter in the construction line)
 * are 40s.
 */
{
    /*
     * Motor controllers
     * Variable names may be changed
     */
    private CANSparkMax m_climb1 = new CANSparkMax(Constants.ClimbConstants.CLIMB_MOTOR1,
            MotorType.kBrushless);

    /**
     * Encoders - assumed to be relative, subject to change
     * Encoders are initialized in the constructor with the helper method
     * "initializeMotor"
     */
    private RelativeEncoder m_encoder1;

    /**
     * Public constructor to be invoked in RobotContainer
     */
    public ClimbSubsystem() {
        m_encoder1 = initializeMotor(m_climb1, false);
    }

    /**
     * Helper method that returns an instance of the given motor controller's
     * relative encoder and sets
     * the motor controller's default behaviours.
     * 
     * @param motorController The motor controller to configure.
     * @param inverted        Inversion state of the encoder, true means inverted.
     * @return An instance of the given motor controller's encoder.
     */
    private RelativeEncoder initializeMotor(CANSparkMax p_motorController, boolean inverted) {

        p_motorController.restoreFactoryDefaults();
        p_motorController.setIdleMode(IdleMode.kBrake);
        p_motorController.setInverted(inverted);

        // .setOpenLoopRampRate(int rate) used in 2022 climb - not needed here?

        // .setPositionConversionFactor(DriverConstants.ENCODER_POSITION_CONVERSION);
        // .setVelocityConversionFactor(DriverConstants.ENCODER_VELOCITY_CONVERSION);
        // ^^ May be needed in the future?

        return p_motorController.getEncoder();
    }

    /**
     * Starts the climb motor to the constant determined speed.
     * NOTE: CURRENT CONSTANT IS A PLACEHOLDER VALUE
     */
    public void extendClimber() {
        m_climb1.set(Constants.ClimbConstants.CLIMB_MOTOR_SPEED);
    }

    /**
     * Stops the motor.
     */
    public void stopClimber() {
        m_climb1.stopMotor();
    }

    /**
     * Starts the climb motor to the constant determined speed.
     * 
     * NOTE: current constant is the same as the extend climber,
     * just negative - should there be a separate value?
     */
    public void rectractClimber() {
        m_climb1.set(-Constants.ClimbConstants.CLIMB_MOTOR_SPEED);
    }

    /**
     * Resets encoders
     */
    public void resetEncoders() {
        m_encoder1.setPosition(0);
    }

    /**
     * Getter for the specified encoder's position.
     * 
     * @param p_encoder The encoder to get position from.
     * @return The position of the given encoder's motor.
     */
    public double getEncoderPosition(RelativeEncoder p_encoder) {
        return p_encoder.getPosition();
    }

    /**
     * Getter for the specified encoder's velocity.
     * 
     * @param p_encoder The encoder to get velocity from.
     * @return The velocity of the given encoder's motor.
     */
    public double getEncoderVelocity(RelativeEncoder p_encoder) {
        return p_encoder.getVelocity();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
