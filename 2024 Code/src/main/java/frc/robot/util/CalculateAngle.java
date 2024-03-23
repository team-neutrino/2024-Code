package frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.TreeMap;
import java.awt.geom.Point2D;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.simulation.InterpolationOptimization;

public class CalculateAngle {

    TreeMap<Double, Double> m_angleData = new TreeMap<Double, Double>();

    TreeMap<Double, Double> m_xAxis = new TreeMap<>();
    TreeMap<Double, Double> m_yAxis = new TreeMap<>();

    HashMap<PolarCoord, Double> bilinearMap = new HashMap<>();

    ArrayList<Double> row = new ArrayList<>();
    ArrayList<Double> col = new ArrayList<>();

    InterpolationOptimization optimizer = new InterpolationOptimization();

    ArrayList<PolarCoord> currentSquare = new ArrayList<>();

    PolarCoord currentRobotPoint;

    PolarCoord p1 = new PolarCoord(10.0, Math.PI); // far bottom left point, initialization for evaluation
    PolarCoord p2;
    PolarCoord p3;
    PolarCoord p4;

    double theta;
    double radius;

    double[] changeAmt = { -0.5, 0.5, -1, 1, -3, 3 };

    double pi = Math.PI;

    public CalculateAngle() {

        bilinearMap.put(new PolarCoord(1.625, 0), 1.22654);
        bilinearMap.put(new PolarCoord(2.0, 0), 5.0);
        bilinearMap.put(new PolarCoord(2.3, 0), 6.34);
        bilinearMap.put(new PolarCoord(2.55, 0), 8.733);
        bilinearMap.put(new PolarCoord(2.8, 0), 11.667);
        bilinearMap.put(new PolarCoord(3.2, 0), 12.632);
        bilinearMap.put(new PolarCoord(3.5, 0), 13.336);
        bilinearMap.put(new PolarCoord(4.0, 0), 15.0);

        bilinearMap.put(new PolarCoord(1.625, pi / 24), 0.004);
        bilinearMap.put(new PolarCoord(2.0, pi / 24), 6.0); //6.296
        bilinearMap.put(new PolarCoord(2.3, pi / 24), 9.0);
        bilinearMap.put(new PolarCoord(2.55, pi / 24), 10.8); //11.0
        bilinearMap.put(new PolarCoord(2.8, pi / 24), 12.1); //12.538
        bilinearMap.put(new PolarCoord(3.2, pi / 24), 15.1668);
        bilinearMap.put(new PolarCoord(3.5, pi / 24), 12.4633);
        bilinearMap.put(new PolarCoord(4.0, pi / 24), 15.0);

        bilinearMap.put(new PolarCoord(1.625, pi / 12), 1.1097);
        bilinearMap.put(new PolarCoord(2.0, pi / 12), 7.827);
        bilinearMap.put(new PolarCoord(2.3, pi / 12), 9.52675);
        bilinearMap.put(new PolarCoord(2.55, pi / 12), 10.562);
        bilinearMap.put(new PolarCoord(2.8, pi / 12), 13.5); //13.93
        bilinearMap.put(new PolarCoord(3.2, pi / 12), 14.787);
        bilinearMap.put(new PolarCoord(3.5, pi / 12), 15.0);
        bilinearMap.put(new PolarCoord(4.0, pi / 12), 15.05);

        bilinearMap.put(new PolarCoord(1.625, pi / 8), 4.76);
        bilinearMap.put(new PolarCoord(2.0, pi / 8), 7.0); // 8.0
        bilinearMap.put(new PolarCoord(2.3, pi / 8), 9.567);
        bilinearMap.put(new PolarCoord(2.55, pi / 8), 10.8569);
        bilinearMap.put(new PolarCoord(2.8, pi / 8), 12.602);
        bilinearMap.put(new PolarCoord(3.2, pi / 8), 11.2204);
        bilinearMap.put(new PolarCoord(3.5, pi / 8), 15.443);
        bilinearMap.put(new PolarCoord(4.0, pi / 8), 18.0);

        bilinearMap.put(new PolarCoord(1.625, pi / 6), 5.0);
        bilinearMap.put(new PolarCoord(2.0, pi / 6), 8.0); // 8.5
        bilinearMap.put(new PolarCoord(2.3, pi / 6), 9.324);
        bilinearMap.put(new PolarCoord(2.55, pi / 6), 10.12);
        bilinearMap.put(new PolarCoord(2.8, pi / 6), 11.05);
        bilinearMap.put(new PolarCoord(3.2, pi / 6), 12.56);
        bilinearMap.put(new PolarCoord(3.5, pi / 6), 15.2706);
        bilinearMap.put(new PolarCoord(4.0, pi / 6), 20.2317);

        bilinearMap.put(new PolarCoord(1.625, pi / 4.8), 6.0);
        bilinearMap.put(new PolarCoord(2.0, pi / 4.8), 7.7); // 8.5
        bilinearMap.put(new PolarCoord(2.3, pi / 4.8), 8.62);
        bilinearMap.put(new PolarCoord(2.55, pi / 4.8), 7.428);
        bilinearMap.put(new PolarCoord(2.8, pi / 4.8), 12.04);
        bilinearMap.put(new PolarCoord(3.2, pi / 4.8), 15.532);
        bilinearMap.put(new PolarCoord(3.5, pi / 4.8), 15.277);
        bilinearMap.put(new PolarCoord(4.0, pi / 4.8), 15.984);

        bilinearMap.put(new PolarCoord(1.625, pi / 4), 6.5);
        bilinearMap.put(new PolarCoord(2.0, pi / 4), 8.0); // 8.5
        bilinearMap.put(new PolarCoord(2.3, pi / 4), 9.0);
        bilinearMap.put(new PolarCoord(2.55, pi / 4), 10.148);
        bilinearMap.put(new PolarCoord(2.8, pi / 4), 11.165);
        bilinearMap.put(new PolarCoord(3.2, pi / 4), 13.8962);
        bilinearMap.put(new PolarCoord(3.5, pi / 4), 17.231);
        bilinearMap.put(new PolarCoord(4.0, pi / 4), 19.1663);

        for (PolarCoord cur_key : bilinearMap.keySet()) {
            if (!col.contains(cur_key.x)) {
                col.add(cur_key.x);
            }
            if (!row.contains(cur_key.y)) {
                row.add(cur_key.y);
            }
        }
        col.sort(Comparator.naturalOrder());
        row.sort(Comparator.naturalOrder());

        currentSquare.add(p1);
        currentSquare.add(p2);
        currentSquare.add(p3);
        currentSquare.add(p4);
    }

    public double InterpolateAngle(PolarCoord robotPoint) {
        radius = robotPoint.getRadius();
        theta = robotPoint.getTheta();
        int index = optimizer.scheduleFunctionChanges();

        if (index != -1) {

            double deltaY = currentSquare.get(2).getY() - currentSquare.get(0).getY();
            double deltaX = currentSquare.get(1).getX() - currentSquare.get(0).getX();

            double yPart1 = ((currentSquare.get(2).getY() - currentRobotPoint.getY()) / deltaY);
            double yPart2 = ((currentRobotPoint.getY() - currentSquare.get(0).getY()) / deltaY);

            double xPart1 = ((currentSquare.get(1).getX() - currentRobotPoint.getX()) / deltaX);
            double xPart2 = ((currentRobotPoint.getX() - currentSquare.get(0).getX()) / deltaX);

            double coeff1 = yPart1 * xPart1;
            double coeff2 = yPart1 * xPart2;
            double coeff3 = yPart2 * xPart1;
            double coeff4 = yPart2 * xPart2;

            double scalar = changeAmt[index]
                    / (Math.pow(coeff1, 2) + Math.pow(coeff2, 2) + Math.pow(coeff3, 2) + Math.pow(coeff4, 2));

            double value1 = bilinearMap.get(currentSquare.get(0));
            double value2 = bilinearMap.get(currentSquare.get(1));
            double value3 = bilinearMap.get(currentSquare.get(2));
            double value4 = bilinearMap.get(currentSquare.get(3));

            bilinearMap.put(currentSquare.get(0), value1 + coeff1 * scalar);
            bilinearMap.put(currentSquare.get(1), value2 + coeff2 * scalar);
            bilinearMap.put(currentSquare.get(2), value3 + coeff3 * scalar);
            bilinearMap.put(currentSquare.get(3), value4 + coeff4 * scalar);
        }

        // define robot point, this is the point that we are approximating f(x,y) for

        currentRobotPoint = robotPoint;

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
                PolarCoord temp = new PolarCoord(col.get(j), row.get(i));
                if (p1.distance(robotPoint) > temp.distance(robotPoint)) {
                    p1 = temp;
                    rowN = i;
                    colN = j;
                }
            }
        }

        if (robotPoint.getX() > col.get(col.size() - 1) || (robotPoint.getX() < col.get(0))) {
            robotPoint = new PolarCoord(col.get(colN), robotPoint.getY());
        }

        if ((robotPoint.getY() > row.get(row.size() - 1)) || (robotPoint.getY() < row.get(0))) {
            robotPoint = new PolarCoord(robotPoint.getX(), row.get(rowN));
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

        // define the four corners

        p1 = new PolarCoord(x1, y1);
        p2 = new PolarCoord(x2, y1);
        p3 = new PolarCoord(x1, y2);
        p4 = new PolarCoord(x2, y2);

        currentSquare.set(0, p1);
        currentSquare.set(1, p2);
        currentSquare.set(2, p3);
        currentSquare.set(3, p4);

        // define weights (coeff 1-4) and multipy the function value at the four corners
        // by each weight determined by the robot's position

        double deltaY = currentSquare.get(2).getY() - currentSquare.get(0).getY();
        double deltaX = currentSquare.get(1).getX() - currentSquare.get(0).getX();

        double yPart1 = ((currentSquare.get(2).getY() - currentRobotPoint.getY()) / deltaY);
        double yPart2 = ((currentRobotPoint.getY() - currentSquare.get(0).getY()) / deltaY);

        double xPart1 = ((currentSquare.get(1).getX() - currentRobotPoint.getX()) / deltaX);
        double xPart2 = ((currentRobotPoint.getX() - currentSquare.get(0).getX()) / deltaX);

        double coeff1 = yPart1 * xPart1;
        double coeff2 = yPart1 * xPart2;
        double coeff3 = yPart2 * xPart1;
        double coeff4 = yPart2 * xPart2;

        double value1 = bilinearMap.get(currentSquare.get(0));
        double value2 = bilinearMap.get(currentSquare.get(1));
        double value3 = bilinearMap.get(currentSquare.get(2));
        double value4 = bilinearMap.get(currentSquare.get(3));

        double result = value1 * coeff1 + value2 * coeff2 + value3 * coeff3 + value4 * coeff4;

        p1 = new PolarCoord(10.0, Math.PI);

        return result;
    }

    /**
     * Calculates set angle for arm while shooting using a best fit kinda wacky
     * plane instead of piecewise bilinear interpolation
     * 
     * @param robotPoint
     * @return
     */
    public double bestFitCalculateAngle(PolarCoord robotPoint) {
        Point2D.Double robotPointxy = new Point2D.Double(robotPoint.getRadius() * Math.cos(robotPoint.getTheta()),
                robotPoint.getRadius() * Math.sin(robotPoint.getTheta()));

        if (robotPointxy.getY() < 0.25) {
            return robotPointxy.getX() * 4;
        }

        return 4.59 * Math.pow(robotPointxy.getX(), 1.09) * Math.pow(robotPointxy.getY(), 0.15);
    }

    /**
     * This method is used for tuning and is required for acquring the modified
     * function value set
     */
    public void dumpData() {
        for (int i = 0; i < row.size(); i++) {
            for (int j = 0; j < col.size(); j++) {
                Point2D.Double temp = new Point2D.Double(col.get(j), row.get(i));
                System.out.println(bilinearMap.get(temp));
            }
        }
        System.out.println("current robot location: r " + currentRobotPoint.getRadius() + " theta "
                + currentRobotPoint.getTheta());
    }

    public double getTheta() {
        return theta;
    }

    public double getRadius() {
        return radius;
    }
}