package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;
import java.util.TreeMap;

import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.simulation.InterpolationOptimization;

public class CalculateAngle {

    TreeMap<Double, Double> m_angleData = new TreeMap<Double, Double>();
    LimelightSubsystem m_limelight;

    TreeMap<Double, Double> m_xAxis = new TreeMap<>();
    TreeMap<Double, Double> m_yAxis = new TreeMap<>();

    HashMap<Point2D.Double, Double> bilinearMap = new HashMap<>();

    ArrayList<Double> row = new ArrayList<>();
    ArrayList<Double> col = new ArrayList<>();

    InterpolationOptimization optimizer = new InterpolationOptimization();

    ArrayList<Point2D.Double> currentSquare = new ArrayList<>();

    Point2D currentRobotPoint;

    Point2D.Double p1 = new Point2D.Double(-10.0, -10.0); // far bottom left point, initialization for evaluation
    Point2D.Double p2;
    Point2D.Double p3;
    Point2D.Double p4;

    double[] changeAmt = { -0.5, 0.5, -1, 1, -2, 2 };

    // Scanner scanner;
    // File inputFile;
    // BufferedWriter writer;

    // // String row1 = "";
    // // String row2 = "";
    // // String row3 = "";
    // // String row4 = "";
    // // String row5 = "";

    // String[] bilinearDataStringArr = { "", "", "", "", "" };
    // String[] bilinearDataStringArr = new String[5];

    public CalculateAngle() {
        m_limelight = SubsystemContainer.limelightSubsystem;
        // File path = new File("BilinearData.txt");
        // String absPath = path.getAbsolutePath();
        // inputFile = new File("/home/lvuser/BilinearData.txt");
        // try {
        // inputFile = new File(
        // Thread.currentThread().getContextClassLoader().getResource("BilinearData.txt").toURI());

        // } catch (URISyntaxException e) {
        // e.printStackTrace();
        // }
        // System.out.println("current dir: " + System.getProperty("user.dir"));

        // File curdir = new File("/home/lvuser/deploy");

        // File[] files = curdir.listFiles();
        // for (File f : files) {
        // System.out.println(f.getName());
        // }

        // try {
        // scanner = new Scanner(inputFile);
        // } catch (FileNotFoundException e) {
        // e.printStackTrace();
        // }

        // scanner.useDelimiter(" ");

        // try
        // {
        // writer = new BufferedWriter(new FileWriter(inputFile));
        // }

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

        // m_xAxis.put(1.26, -8.17);
        // m_xAxis.put(1.688, -3.25);
        // m_xAxis.put(2.11, 0.5);
        // m_xAxis.put(2.42, 6.3);
        // // 2.78, higher angle (adjusted)
        // m_xAxis.put(2.5, 9.25);
        // m_xAxis.put(2.8, 10.0);
        // m_xAxis.put(3.12, 10.2);
        // m_xAxis.put(3.24, 10.7);
        // m_xAxis.put(3.29, 12.13);
        // m_xAxis.put(3.7, 13.0);

        // m_yAxis.put(0.0, 0.0);
        // m_yAxis.put(0.5, 2.75);
        // m_yAxis.put(1.0, 4.5);
        // m_yAxis.put(1.7, 5.5);
        // m_yAxis.put(2.5, 6.5);

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

        bilinearMap.put(new Point2D.Double(1.625, 0), 1.22654);
        bilinearMap.put(new Point2D.Double(2.3, 0), 5.0);
        bilinearMap.put(new Point2D.Double(2.8, 0), 11.5);
        bilinearMap.put(new Point2D.Double(3.5, 0), 15.2); // 12 in auton testing
        bilinearMap.put(new Point2D.Double(4.0, 0.0), 16.0);
        bilinearMap.put(new Point2D.Double(1.625, 0.85), 2.07);

        bilinearMap.put(new Point2D.Double(2.3, 0.85), 15.0);
        bilinearMap.put(new Point2D.Double(2.8, 0.85), 14.64);
        bilinearMap.put(new Point2D.Double(3.5, 0.85), 18.31);
        bilinearMap.put(new Point2D.Double(4.0, 0.85), 20.25);
        bilinearMap.put(new Point2D.Double(1.625, 1.2), 5.73);

        bilinearMap.put(new Point2D.Double(2.3, 1.2), 10.81);
        bilinearMap.put(new Point2D.Double(2.8, 1.2), 14.4);

        bilinearMap.put(new Point2D.Double(3.5, 1.2), 18.7);
        bilinearMap.put(new Point2D.Double(4.0, 1.2), 20.5);

        bilinearMap.put(new Point2D.Double(1.625, 2.13), 8.953933);
        bilinearMap.put(new Point2D.Double(2.3, 2.13), 12.84);
        bilinearMap.put(new Point2D.Double(2.8, 2.13), 17.25);
        bilinearMap.put(new Point2D.Double(3.5, 2.13), 21.0);
        bilinearMap.put(new Point2D.Double(4.0, 2.13), 21.8);

        bilinearMap.put(new Point2D.Double(1.625, 2.7), 15.0);
        bilinearMap.put(new Point2D.Double(2.3, 2.7), 17.0);
        bilinearMap.put(new Point2D.Double(2.8, 2.7), 18.48);
        bilinearMap.put(new Point2D.Double(3.5, 2.7), 20.40);
        bilinearMap.put(new Point2D.Double(4.0, 2.7), 21.6);

        // bilinearMap.put(new Point2D.Double(2.8, 1.2), 14.4); // 11.6 in auton testing

        // bilinearMap.put(new Point2D.Double(3.5, 2.13), 19.0);

        // bilinearMap.put(new Point2D.Double(2.3, 2.7), 18.0);
        // bilinearMap.put(new Point2D.Double(2.8, 2.13), 17.25);

        col.add(1.625);
        col.add(2.3);
        col.add(2.8);
        col.add(3.5);
        col.add(4.0);
        // col.add(4.5);

        row.add(0.0);
        row.add(0.85);
        row.add(1.2);
        row.add(2.13);
        row.add(2.7);

        // String temp;

        // for (int i = 0; i < row.size(); i++) {
        // for (int j = 0; j < col.size(); j++) {
        // temp = scanner.next();
        // bilinearMap.put(new Point2D.Double(col.get(j), row.get(i)),
        // Double.valueOf(temp));

        // // bilinearDataStringArr[i].concat(temp + " ");
        // }
        // // bilinearDataStringArr[i].concat("\n");
        // }

        // scanner.close();

        currentSquare.add(p1);
        currentSquare.add(p2);
        currentSquare.add(p3);
        currentSquare.add(p4);
    }

    public double InterpolateAngle() {

        int index = optimizer.scheduleFunctionChanges();

        if (index != -1) {
            double scalar = changeAmt[index];

            double deltaY = currentSquare.get(2).getY() - currentSquare.get(0).getY();
            double deltaX = currentSquare.get(1).getX() - currentSquare.get(0).getX();

            double yPart1 = ((currentSquare.get(2).getY() - currentRobotPoint.getY()) / deltaY);
            double yPart2 = ((currentRobotPoint.getY() - currentSquare.get(0).getY()) / deltaY);

            double coeff1 = yPart1 * ((currentSquare.get(1).getX() - currentRobotPoint.getX()) / deltaX);
            double coeff2 = yPart1 * ((currentRobotPoint.getX() - currentSquare.get(0).getX()) / deltaX);
            double coeff3 = yPart2 * ((currentSquare.get(1).getX() - currentRobotPoint.getX()) / deltaX);
            double coeff4 = yPart2 * ((currentRobotPoint.getX() - currentSquare.get(0).getX()) / deltaX);

            // for (int i = 0; i < currentSquare.size(); i++)
            // {
            // bilinearMap.put(currentSquare.get(i), bilinearMap.get(currentSquare.get(i)) *
            // co)
            // }

            System.out.println("change 1: " + coeff1 * scalar);
            System.out.println("change 2: " + coeff2 * scalar);
            System.out.println("change 3: " + coeff3 * scalar);
            System.out.println("index " + index);

            double value1 = bilinearMap.get(currentSquare.get(0));
            double value2 = bilinearMap.get(currentSquare.get(1));
            double value3 = bilinearMap.get(currentSquare.get(2));
            double value4 = bilinearMap.get(currentSquare.get(3));

            System.out.println(value1);

            bilinearMap.put(currentSquare.get(0), value1 + coeff1 * scalar);
            bilinearMap.put(currentSquare.get(1), value2 + coeff2 * scalar);
            bilinearMap.put(currentSquare.get(2), value3 + coeff3 * scalar);
            bilinearMap.put(currentSquare.get(3), value4 + coeff4 * scalar);

            // String temp;

            // for (int i = 0; i < row.size(); i++) {
            // for (int j = 0; j < col.size(); j++) {
            // temp = String.valueOf(bilinearMap.get(new Point2D.Double(col.get(j),
            // row.get(i))));

            // bilinearDataStringArr[i] = bilinearDataStringArr[i].concat(temp + " ");
            // }
            // bilinearDataStringArr[i] = bilinearDataStringArr[i].concat("\n");
            // }

            // try {
            // writer = new BufferedWriter(new FileWriter(inputFile));
            // for (int i = 0; i < bilinearDataStringArr.length; i++) {
            // writer.write(bilinearDataStringArr[i]);
            // bilinearDataStringArr[i] = "";
            // }
            // writer.close();
            // } catch (IOException e) {
            // e.printStackTrace();
            // }

            System.out.println("successfully overwrote bilinearData.txt");

            // for (int i = 0; i < bilinearDataStringArr.length; i++)
            // {
            // bilinearDataStringArr[i] = "";
            // }
        }

        // Double smallX = 0.0;
        // Double largeX = 0.0;
        // Double smallY = 0.0;
        // Double largeY = 0.0;
        // double resultX = 0;
        // double resultY = 0;
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

        currentRobotPoint = robotPoint;

        System.out.println("robot point x: " + robotPoint.getX());
        System.out.println("robot point y " + robotPoint.getY());

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

        // else if (col.get(colN) < robotPoint.getX())
        // {
        // robotPoint = new Point2D.Double(col.get(colN), robotPoint.getY());

        // }

        // if (col.get(colN) == col.get(0) || col.get(colN) == col.get(3) &&
        // row.get(rowN) == row.get(0) || row.get(rowN) == row.get(4))
        // {

        // }

        System.out.println("coln " + colN);
        System.out.println("rown " + rowN);

        if (robotPoint.getX() > col.get(col.size() - 1) || (robotPoint.getX() < col.get(0))) {
            robotPoint = new Point2D.Double(col.get(colN), robotPoint.getY());
        }

        if ((robotPoint.getY() > row.get(row.size() - 1)) || (robotPoint.getY() < row.get(0))) {
            robotPoint = new Point2D.Double(robotPoint.getX(), row.get(rowN));
        }

        if (colN == col.size() - 1 || robotPoint.getX() < col.get(colN)) {
            x2 = col.get(colN);
            x1 = col.get(colN - 1);
        } else {
            x1 = col.get(colN);
            x2 = col.get(colN + 1);
        }

        if (rowN == row.size() - 1 || robotPoint.getY() < row.get(rowN)) {
            y2 = row.get(rowN);
            y1 = row.get(rowN - 1);
        } else {
            y1 = row.get(rowN);
            y2 = row.get(rowN + 1);
        }

        System.out.println("x1 " + x1 + " x2 " + x2 + " y1 " + y1 + " y2 " + y2);

        // define the four corners

        p1 = new Point2D.Double(x1, y1);
        p2 = new Point2D.Double(x2, y1);
        p3 = new Point2D.Double(x1, y2);
        p4 = new Point2D.Double(x2, y2);

        currentSquare.set(0, p1);
        currentSquare.set(1, p2);
        currentSquare.set(2, p3);
        currentSquare.set(3, p4);

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

        p1 = new Point2D.Double(-10.0, -10.0);

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

    public void dumpData() {
        for (int i = 0; i < row.size(); i++) {
            for (int j = 0; j < col.size(); j++) {
                Point2D.Double temp = new Point2D.Double(col.get(j), row.get(i));
                System.out.println(bilinearMap.get(temp));
            }
        }
    }
}