package frc.robot.util;

import java.util.TreeMap;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class CalculateAngle {

    TreeMap<Double, Double> m_angleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    public CalculateAngle() {
        m_limelight = SubsystemContainer.limelightSubsystem;

        // m_angleData.put(11.97, -13.53);
        // m_angleData.put(8.64, -13.48);
        // m_angleData.put(3.5, -10.429);
        // m_angleData.put(0.0, -4.029);
        // m_angleData.put(-3.0, -4.0);
        // m_angleData.put(-6.0, -1.7);
        // m_angleData.put(-8.55, 3.0);
        // m_angleData.put(-10.0, 5.0);
        // m_angleData.put(-12.07, 7.2);
        // m_angleData.put(-14.03, 8.0);
        // m_angleData.put(-16.0, 9.5);
        m_angleData.put(1.26, -8.17);
        m_angleData.put(1.688, -2.57);
        m_angleData.put(2.11, 1.65);
        m_angleData.put(2.42, 5.32);
        // 2.78, higher angle (adjusted)
        m_angleData.put(2.8, 7.5);
        m_angleData.put(3.24, 10.7);
        m_angleData.put(3.29, 11.0);
        m_angleData.put(3.7, 11.13);
    }

    public double InterpolateAngle() {
        Double smallerAngle = 0.0;
        Double largerAngle = 0.0;
        double resultAngle = 0;
        // double ty = m_limelight.getTy();

        double distance = Math.sqrt(Math.pow(
                SubsystemContainer.swerveSubsystem.currentPoseL.getX() - SwerveConstants.SPEAKER_BLUE_SIDE.getX(), 2)
                + Math.pow(SubsystemContainer.swerveSubsystem.currentPoseL.getY()
                        - SwerveConstants.SPEAKER_BLUE_SIDE.getY(), 2));

        smallerAngle = m_angleData.lowerKey(distance);
        largerAngle = m_angleData.higherKey(distance);

        if (smallerAngle == null) {
            smallerAngle = m_angleData.firstKey();
        } else if (largerAngle == null) {
            largerAngle = m_angleData.lastKey();
        }

        resultAngle = (m_angleData.get(smallerAngle) * (largerAngle - distance)
                + m_angleData.get(largerAngle) * (distance - smallerAngle))
                / (largerAngle - smallerAngle);

        return resultAngle;
    }
}