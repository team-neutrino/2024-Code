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
 * 1/19 it is still unclear how extending the
 * spring and winch based arms will work on the controls
 * side, locking the mechanism in place during teleop,
 * how many motors the configuration will have (2 or 4),
 * and if limits will be needed.
 * 
 * despite not having an "extend arms" command on the assumption
 * that it will not be necessary with the current spring model (see 2022),
 * both the constant for arm extension motor speed and method for
 * arm extension have been kept in case they are needed.
 */
public class ClimbSubsystem extends SubsystemBase {
    /*
     * Motor controllers
     * Variable names may be changed
     */
    private CANSparkMax m_climbArm1 = new CANSparkMax(Constants.MotorIDs.CLIMB_MOTOR1,
            MotorType.kBrushless);
    private CANSparkMax m_climbArm2 = new CANSparkMax(Constants.MotorIDs.CLIMB_MOTOR2, MotorType.kBrushless);

    /**
     * Encoders - assumed to be relative, subject to change
     * Encoders are initialized in the constructor with the helper method
     * "initializeMotor"
     */
    private RelativeEncoder m_armEncoder1;
    private RelativeEncoder m_armEncoder2;

    /**
     * Public constructor to be invoked in RobotContainer
     */
    public ClimbSubsystem() {
        m_armEncoder1 = initializeMotor(m_climbArm1, false);
        m_armEncoder2 = initializeMotor(m_climbArm2, false);
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
     * Starts the arm motors to the constant determined extend speed.
     * NOTE: CURRENT CONSTANT IS A PLACEHOLDER VALUE
     * NOTE: THIS METHOD MAY NOT BE NEEDED WHEN MORE DETAILS ARE KNOWN
     */
    public void extendClimberArms() {
        m_climbArm1.set(Constants.ClimbConstants.CLIMB_EXTEND_MOTOR_SPEED);
        m_climbArm2.set(Constants.ClimbConstants.CLIMB_EXTEND_MOTOR_SPEED);
    }

    /**
     * Stops the arm motors.
     */
    public void stopClimberArms() {
        m_climbArm1.stopMotor();
        m_climbArm2.stopMotor();
    }

    /**
     * Starts the arm motors to the constant determined retract speed.
     * 
     * TODO: current constant is the same as the extend climber,
     * just negative - should there be a separate value?
     */
    public void rectractClimberArms() {
        m_climbArm1.set(Constants.ClimbConstants.CLIMB_RETRACT_MOTOR_SPEED);
        m_climbArm2.set(Constants.ClimbConstants.CLIMB_RETRACT_MOTOR_SPEED);
    }

    /**
     * Resets all encoders
     */
    public void resetEncoders() {
        m_armEncoder1.setPosition(0);
        m_armEncoder2.setPosition(0);
    }

    /**
     * Getter for the arm motors' positions.
     * 
     * @return An array of the arm motors' positions.
     */
    public double[] getArmEncoderPosition() {
        return new double[] { m_armEncoder1.getPosition(), m_armEncoder2.getPosition() };
    }

    /**
     * Getter for the arm motors' velocities.
     * 
     * @return An array of the arm motors' velocities.
     */

    public double[] getArmEncoderVelocity() {
        return new double[] { m_armEncoder1.getVelocity(), m_armEncoder2.getVelocity() };
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
