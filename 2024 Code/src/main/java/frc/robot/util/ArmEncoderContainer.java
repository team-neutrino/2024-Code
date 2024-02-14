package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.simulation.ArmSimulation;

public class ArmEncoderContainer {
    public SparkAbsoluteEncoder m_armEncoder;

    public ArmEncoderContainer(SparkAbsoluteEncoder p_armEncoder) {
        m_armEncoder = p_armEncoder;
    }

    public int getAverageDepth() {
        return m_armEncoder.getAverageDepth();
    }

    public boolean getInverted() {
        return m_armEncoder.getInverted();
    }

    public double getPosition() {
        if (RobotBase.isSimulation()) {
            return ArmSimulation.currentSimAngle;
        } else {
            return m_armEncoder.getPosition();
        }
    }

    public double getVelocity() {
        return m_armEncoder.getVelocity();
    }

    public double getZeroOffset() {
        return m_armEncoder.getZeroOffset();
    }

    public REVLibError setAverageDepth(int depth) {
        return m_armEncoder.setAverageDepth(depth);
    }

    public REVLibError setInverted(boolean inverted) {
        return m_armEncoder.setInverted(inverted);
    }

    public REVLibError setPositionConversionFactor(double factor) {
        return m_armEncoder.setPositionConversionFactor(factor);
    }

    public REVLibError setVelocityConversionFactor(double factor) {
        return m_armEncoder.setVelocityConversionFactor(factor);
    }

    public REVLibError setZeroOffset(double offset) {
        return m_armEncoder.setZeroOffset(offset);
    }
}
