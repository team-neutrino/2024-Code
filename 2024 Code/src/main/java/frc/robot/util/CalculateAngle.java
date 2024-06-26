package frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.TreeMap;
import java.awt.geom.Point2D;

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

        bilinearMap.put(new PolarCoord(2.15, 0), 9.866);
        bilinearMap.put(new PolarCoord(3.1, 0), 15.354);
        bilinearMap.put(new PolarCoord(4.0, 0.0), 16.0);
        bilinearMap.put(new PolarCoord(4.9, 0.0), 16.9);

        bilinearMap.put(new PolarCoord(2.15, pi / 8), 7.471);
        bilinearMap.put(new PolarCoord(3.1, pi / 8), 10.56);
        bilinearMap.put(new PolarCoord(4.0, pi / 8), 15.0);
        bilinearMap.put(new PolarCoord(4.9, pi / 8), 17.28);

        bilinearMap.put(new PolarCoord(2.15, pi / 4), 7.0);
        bilinearMap.put(new PolarCoord(3.1, pi / 4), 12.7926);
        bilinearMap.put(new PolarCoord(4.0, pi / 4), 16.6866);
        bilinearMap.put(new PolarCoord(4.9, pi / 4), 18.133);

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
     * plane instead of piecewise bilinear interpolation (it's some exponential
     * thing idk...)
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

    public double quarticFitCalculateAngle(PolarCoord robotPoint) {
        double x = robotPoint.getRadius() * Math.cos(robotPoint.getTheta());
        double y = robotPoint.getRadius() * Math.sin(robotPoint.getTheta());

        return 0.1590165304679383 - 29.69482794140633 * x + 45.19374530251578 * y + 30.083099935511076 * Math.pow(x, 2)
                - 26.76490228955897 * x * y - 19.88973 * Math.pow(y, 2) - 8.83057607244 * Math.pow(x, 3)
                + 5.910997777 * Math.pow(x, 2) * y
                + 4.007635 * Math.pow(y, 2) * x + 7.5481769 * Math.pow(y, 3) + 0.8490822 * Math.pow(x, 4) -
                0.574701463453621 * Math.pow(x, 3) * y + 0.6847081014154278 * Math.pow(x, 2) * Math.pow(y, 2)
                - 2.2305911328185593 * x * Math.pow(y, 3) -
                0.10198575174497826 * Math.pow(y, 4);
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