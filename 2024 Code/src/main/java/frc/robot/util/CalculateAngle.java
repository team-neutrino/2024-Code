package frc.robot.util;

import java.util.ArrayList;
import java.util.TreeMap;

import java.awt.geom.Point2D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class CalculateAngle {

    TreeMap<Double, Double> m_angleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    TreeMap<Double, Double> m_xAxis = new TreeMap<>();
    TreeMap<Double, Double> m_yAxis = new TreeMap<>();

    TreeMap<Point2D.Double, Double> bilinearMap = new TreeMap<>();

    ArrayList<Double> row = new ArrayList<>();
    ArrayList<Double> col = new ArrayList<>();

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

        // bilinearMap.put(new Point2D.Double(1.71, 0.89), -2.0);
        // bilinearMap.put(new Point2D.Double(1.625, 1.97), 7.0);
        // bilinearMap.put(new Point2D.Double(1.55, 2.8), 13.5);
        // bilinearMap.put(new Point2D.Double(2.3, 2.03), 13.4);
        // bilinearMap.put(new Point2D.Double(2.37, 1.38), 10.5);
        // bilinearMap.put(new Point2D.Double(2.384, 0.82), 10.3);
        // bilinearMap.put(new Point2D.Double(2.829, 1.144), 10.4);
        // bilinearMap.put(new Point2D.Double(3.58, 1.34), 14.0);
        // bilinearMap.put(new Point2D.Double(3.46, 2.24), 15.0);
        // bilinearMap.put(new Point2D.Double(2.19, 3.12), 15.2);
        // //bilinearMap.put(new Point2D.Double(3.23, 3.2), 16.3); I have it crossed out
        // in my notes...
        // bilinearMap.put(new Point2D.Double(3.3, 2.259), 14.7);
        // bilinearMap.put(new Point2D.Double(2.75, 2.6044), 14.3);
        // bilinearMap.put(new Point2D.Double(2.53, 1.19), 11.0);

        bilinearMap.put(new Point2D.Double(2.3, 0.85), 10.3);
        bilinearMap.put(new Point2D.Double(1.625, 0.85), -2.0);
        bilinearMap.put(new Point2D.Double(2.3, 1.2), 10.75);
        bilinearMap.put(new Point2D.Double(2.8, 1.2), 10.4);
        bilinearMap.put(new Point2D.Double(1.625, 1.2), 2.5);
        bilinearMap.put(new Point2D.Double(1.625, 2.13), 8.0);
        bilinearMap.put(new Point2D.Double(2.3, 2.13), 13.4);
        bilinearMap.put(new Point2D.Double(1.625, 2.7), 11.0);
        bilinearMap.put(new Point2D.Double(2.3, 2.7), 12.5);
        bilinearMap.put(new Point2D.Double(2.8, 1.2), 10.4);
        bilinearMap.put(new Point2D.Double(3.5, 1.2), 14.0);
        bilinearMap.put(new Point2D.Double(3.5, 2.13), 15.0);
        bilinearMap.put(new Point2D.Double(2.8, 2.7), 14.3);
        bilinearMap.put(new Point2D.Double(2.3, 2.7), 13.8);
        bilinearMap.put(new Point2D.Double(1.625, 0), -2.57);
        bilinearMap.put(new Point2D.Double(2.3, 0), 4.5);
        bilinearMap.put(new Point2D.Double(2.8, 0), 6.9);
        bilinearMap.put(new Point2D.Double(3.5, 0), 10.9);
        bilinearMap.put(new Point2D.Double(2.8, 2.13), 13.75);
        bilinearMap.put(new Point2D.Double(2.8, 0.85), 10.35);
        bilinearMap.put(new Point2D.Double(3.5, 0.85), 13.85);
        bilinearMap.put(new Point2D.Double(3.5, 2.13), 15.0);
        bilinearMap.put(new Point2D.Double(3.5, 2.7), 15.6);

        col.add(1.625);
        col.add(1.625);
        col.add(2.8);
        col.add(3.5);

        row.add(0.0);
        row.add(0.85);
        row.add(1.2);
        row.add(2.13);
        row.add(2.7);
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

        // Point2D[] points = (Point2D[]) bilinearMap.keySet().toArray();

        double xComp;
        double yComp;

        if (SubsystemContainer.swerveSubsystem.isRedAlliance) {
            xComp = Math.abs(
                    SubsystemContainer.swerveSubsystem.currentPoseL.getX() - SwerveConstants.SPEAKER_RED_SIDE.getX());
            yComp = Math.abs(
                    SubsystemContainer.swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_RED_SIDE.getY());
        } else {
            xComp = Math.abs(SubsystemContainer.swerveSubsystem.currentPoseL.getX());
            yComp = Math.abs(
                    SubsystemContainer.swerveSubsystem.currentPoseL.getY() - SwerveConstants.SPEAKER_BLUE_SIDE.getY());
        }

        // define robot point, this is the point that we are approximating f(x,y) for

        Point2D robotPoint = new Point2D.Double(xComp, yComp);

        Point2D p1 = new Point2D.Double(1.625, 0.85); // far bottom left point, initialization for evaluation
        Point2D p2;
        Point2D p3;
        Point2D p4;

        // for (int i = 0; i < points.length; i++)
        // {
        // int temp = i;
        // for (int j = i; j < points.length; j++)
        // {
        // if (points[temp].distance(robotPoint) > points[j].distance(robotPoint))
        // {
        // temp = j;
        // }
        // p1 = points[i];
        // points[i] = points[temp];
        // points[temp] = p1;
        // }
        // }

        int rowN = 0;
        int colN = 0;

        double x1;
        double x2;
        double y1;
        double y2;

        // determine which "box" this point is in by first finding the closest point and
        // then
        // determining which sides will enclose the point depending on where the closest
        // point falls

        for (int i = 0; i < row.size(); i++) {
            for (int j = 0; j < col.size(); j++) {
                Point2D.Double temp = new Point2D.Double(col.get(j), row.get(i));
                if (p1.distance(robotPoint) > temp.distance(robotPoint)) {
                    p1 = temp;
                    rowN = i;
                    colN = j;
                }
            }
        }

        if (col.get(colN) > robotPoint.getX()) {
            x2 = col.get(colN);
            x1 = col.get(colN - 1);
        } else {
            x1 = col.get(colN);
            x2 = col.get(colN + 1);
        }

        if (row.get(rowN) > robotPoint.getY()) {
            y2 = row.get(rowN);
            y1 = row.get(rowN - 1);
        } else {
            y1 = row.get(rowN);
            y2 = row.get(rowN + 1);
        }

        // define the four corners

        p1 = new Point2D.Double(x1, y1);
        p2 = new Point2D.Double(x2, y1);
        p3 = new Point2D.Double(x1, y2);
        p4 = new Point2D.Double(x2, y2);

        // define the three matrices that are needed for the computation,
        // one stores the values of the function at each point, one stores
        // some delta y terms, the other stores corrosponding delta x terms

        Matrix<N2, N2> zMatrix = new Matrix<N2, N2>(Nat.N2(), Nat.N2());

        zMatrix.set(0, 0, bilinearMap.get(p1));
        zMatrix.set(0, 1, bilinearMap.get(p2));
        zMatrix.set(1, 0, bilinearMap.get(p3));
        zMatrix.set(1, 1, bilinearMap.get(p4));

        Matrix<N2, N1> deltaYMatrix = new Matrix<N2, N1>(Nat.N2(), Nat.N1());

        deltaYMatrix.set(0, 0, y2 - robotPoint.getY());
        deltaYMatrix.set(1, 0, robotPoint.getY() - y1);

        Matrix<N1, N2> deltaXMatrix = new Matrix<N1, N2>(Nat.N1(), Nat.N2());

        deltaXMatrix.set(0, 0, x2 - robotPoint.getX());
        deltaXMatrix.set(0, 1, robotPoint.getX() - x1);

        // carry out computation

        Matrix<N1, N2> productOne = deltaXMatrix.times(zMatrix);

        Matrix<N1, N1> productTwo = productOne.times(deltaYMatrix);

        double result = productTwo.get(0, 0) / ((x2 - x1) * (y2 - y1));

        // smallX = m_xAxis.lowerKey(xComp);
        // largeX = m_xAxis.higherKey(xComp);

        // smallY = m_yAxis.lowerKey(yComp);
        // largeY = m_yAxis.higherKey(yComp);

        // if (smallX == null) {
        // smallX = m_xAxis.firstKey();
        // } else if (largeX == null) {
        // largeX = m_xAxis.lastKey();
        // }

        // if (smallY == null) {
        // smallY = m_yAxis.firstKey();
        // } else if (largeX == null) {
        // largeY = m_yAxis.lastKey();
        // }

        // resultX = (m_xAxis.get(smallX) * (largeX - xComp)
        // + m_xAxis.get(largeX) * (xComp - smallX))
        // / (largeX - smallX);

        // resultY = (m_yAxis.get(smallY) * (largeY - yComp)
        // + m_yAxis.get(largeY) * (yComp - smallY))
        // / (largeY - smallY);

        // if (Double.isNaN(resultY)) {
        // resultY = 0.0;
        // System.out.println("current y is NaN");
        // }

        return result;
    }
}