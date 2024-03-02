package frc.robot.util;

import java.util.TreeMap;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class CalculateAngle {

    TreeMap<Double, Double> m_angleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    TreeMap<Double, Double> m_xAxis = new TreeMap<>();
    TreeMap<Double, Double> m_yAxis = new TreeMap<>();

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
        // m_xAxis.put(1.26, -9.0);
        // m_xAxis.put(1.688, -6.25);
        // m_xAxis.put(2.11, 2.25);
        // m_xAxis.put(2.42, 7.25);
        // // 2.78, higher angle (adjusted)
        // m_xAxis.put(2.5, 10.25);
        // m_xAxis.put(2.8, 12.0);
        // m_xAxis.put(3.12, 12.2);
        // m_xAxis.put(3.24, 12.7);
        // m_xAxis.put(3.29, 13.0);
        // m_xAxis.put(3.7, 13.13);

        m_xAxis.put(1.26, -8.17);
        m_xAxis.put(1.688, -3.25);
        m_xAxis.put(2.11, 0.5);
        m_xAxis.put(2.42, 6.3);
        // 2.78, higher angle (adjusted)
        m_xAxis.put(2.5, 9.25);
        m_xAxis.put(2.8, 10.0);
        m_xAxis.put(3.12, 10.2);
        m_xAxis.put(3.24, 10.7);
        m_xAxis.put(3.29, 12.13);
        m_xAxis.put(3.7, 13.0);

        m_yAxis.put(0.0, 0.0);
        m_yAxis.put(0.5, 2.75);
        m_yAxis.put(1.0, 4.5);
        m_yAxis.put(1.7, 5.5);
        m_yAxis.put(2.5, 6.5);
    }

    public double InterpolateAngle() {
        Double smallX = 0.0;
        Double largeX = 0.0;
        Double smallY = 0.0;
        Double largeY = 0.0;
        double resultX = 0;
        double resultY = 0;
        // double[] botPose = SubsystemContainer.limelightSubsystem.getBotPose();
        // double out = 0;
        // double ty = m_limelight.getTy();

        // double distance = Math.sqrt(Math.pow(
        // SubsystemContainer.swerveSubsystem.currentPoseL.getX() -
        // SwerveConstants.SPEAKER_BLUE_SIDE.getX(), 2)
        // + Math.pow(SubsystemContainer.swerveSubsystem.currentPoseL.getY()
        // - SwerveConstants.SPEAKER_BLUE_SIDE.getY(), 2));

        double xComp = Math
                .abs(SubsystemContainer.swerveSubsystem.currentPoseL.getX());
        double yComp = Math
                .abs(SubsystemContainer.swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY());

        smallX = m_xAxis.lowerKey(xComp);
        largeX = m_xAxis.higherKey(xComp);

        smallY = m_yAxis.lowerKey(yComp);
        largeY = m_yAxis.higherKey(yComp);

        if (smallX == null) {
            smallX = m_xAxis.firstKey();
        } else if (largeX == null) {
            largeX = m_xAxis.lastKey();
        }

        if (smallY == null) {
            smallY = m_yAxis.firstKey();
        } else if (largeX == null) {
            largeY = m_yAxis.lastKey();
        }

        resultX = (m_xAxis.get(smallX) * (largeX - xComp)
                + m_xAxis.get(largeX) * (xComp - smallX))
                / (largeX - smallX);

        resultY = (m_yAxis.get(smallY) * (largeY - yComp)
                + m_yAxis.get(largeY) * (yComp - smallY))
                / (largeY - smallY);

        if (Double.isNaN(resultY)) {
            resultY = 0.0;
            System.out.println("current y is NaN");
        }

        return resultX + resultY;
    }
}