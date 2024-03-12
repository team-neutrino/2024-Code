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

    public CalculateAngle() {

        bilinearMap.put(new Point2D.Double(1.625, 0), 1.22654);
        bilinearMap.put(new Point2D.Double(2.3, 0), 5.0);
        bilinearMap.put(new Point2D.Double(2.8, 0), 11.5);
        bilinearMap.put(new Point2D.Double(3.5, 0), 14.9); // 12 in auton testing
        bilinearMap.put(new Point2D.Double(4.0, 0.0), 16.0);
        bilinearMap.put(new Point2D.Double(1.625, 0.85), 2.04);

        bilinearMap.put(new Point2D.Double(2.3, 0.85), 14.97);
        bilinearMap.put(new Point2D.Double(2.8, 0.85), 14.35);
        bilinearMap.put(new Point2D.Double(3.5, 0.85), 18.25);
        bilinearMap.put(new Point2D.Double(4.0, 0.85), 20.25);
        bilinearMap.put(new Point2D.Double(1.625, 1.2), 6.45619);

        bilinearMap.put(new Point2D.Double(2.3, 1.2), 15.4825);
        bilinearMap.put(new Point2D.Double(2.8, 1.2), 14.4);

        bilinearMap.put(new Point2D.Double(3.5, 1.2), 19.21);
        bilinearMap.put(new Point2D.Double(4.0, 1.2), 22.0);

        bilinearMap.put(new Point2D.Double(1.625, 2.13), 9.16);
        bilinearMap.put(new Point2D.Double(2.3, 2.13), 20.15);
        bilinearMap.put(new Point2D.Double(2.8, 2.13), 16.6);
        bilinearMap.put(new Point2D.Double(3.5, 2.13), 21.27);
        bilinearMap.put(new Point2D.Double(4.0, 2.13), 22.14);

        bilinearMap.put(new Point2D.Double(1.625, 2.7), 15.0);
        bilinearMap.put(new Point2D.Double(2.3, 2.7), 17.31);
        bilinearMap.put(new Point2D.Double(2.8, 2.7), 20.66);
        bilinearMap.put(new Point2D.Double(3.5, 2.7), 20.40);
        bilinearMap.put(new Point2D.Double(4.0, 2.7), 21.6);

        for ( Point2D.Double cur_key : bilinearMap.keySet() )
        {
            if( !col.contains(cur_key.x) )
            {
                col.add(cur_key.x);
            }
            if( !row.contains(cur_key.y) )
            {
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

    public double InterpolateAngle(Point2D robotPoint) {

        // int index = optimizer.scheduleFunctionChanges();

        // if (index != -1) {

        //     double deltaY = currentSquare.get(2).getY() - currentSquare.get(0).getY();
        //     double deltaX = currentSquare.get(1).getX() - currentSquare.get(0).getX();

        //     double yPart1 = ((currentSquare.get(2).getY() - currentRobotPoint.getY()) / deltaY);
        //     double yPart2 = ((currentRobotPoint.getY() - currentSquare.get(0).getY()) / deltaY);

        //     double xPart1 = ((currentSquare.get(1).getX() - currentRobotPoint.getX()) / deltaX);
        //     double xPart2 = ((currentRobotPoint.getX() - currentSquare.get(0).getX()) / deltaX);

        //     double coeff1 = yPart1 * xPart1;
        //     double coeff2 = yPart1 * xPart2;
        //     double coeff3 = yPart2 * xPart1;
        //     double coeff4 = yPart2 * xPart2;

        //     double scalar = changeAmt[index]
        //             / (Math.pow(coeff1, 2) + Math.pow(coeff2, 2) + Math.pow(coeff3, 2) + Math.pow(coeff4, 2));

        //     double value1 = bilinearMap.get(currentSquare.get(0));
        //     double value2 = bilinearMap.get(currentSquare.get(1));
        //     double value3 = bilinearMap.get(currentSquare.get(2));
        //     double value4 = bilinearMap.get(currentSquare.get(3));

        //     bilinearMap.put(currentSquare.get(0), value1 + coeff1 * scalar);
        //     bilinearMap.put(currentSquare.get(1), value2 + coeff2 * scalar);
        //     bilinearMap.put(currentSquare.get(2), value3 + coeff3 * scalar);
        //     bilinearMap.put(currentSquare.get(3), value4 + coeff4 * scalar);
        // }

        // define robot point, this is the point that we are approximating f(x,y) for

        // currentRobotPoint = robotPoint;
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

        // define the four corners

        p1 = new Point2D.Double(x1, y1);
        p2 = new Point2D.Double(x2, y1);
        p3 = new Point2D.Double(x1, y2);
        p4 = new Point2D.Double(x2, y2);

        System.err.println(p3.x + "," + p3.y + "______________________________________" + p4.x + "," + p4.y);
        System.err.println("|"               + "                                                      " + "|");
        System.err.println("|"               + "                                                      " + "|");
        System.err.println("|"               + "                    " + robotPoint.getX() + "," + robotPoint.getY() +  "                         " + "|");
        System.err.println("|"               + "                                                      " + "|");
        System.err.println("|"               + "                                                      " + "|");
        System.err.println(p1.x + "," + p1.y + "______________________________________" + p2.x + "," + p2.y);

        currentSquare.set(0, p1);
        currentSquare.set(1, p2);
        currentSquare.set(2, p3);
        currentSquare.set(3, p4);

        // define the three matrices that are needed for the computation,
        // one stores the values of the function at each point, one stores
        // some delta y terms, the other stores corresponding delta x terms

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

        // reset point (running into oscillation issues, probably because this point not
        // being reset
        // led to interference with the shortest distance algorithm necessary to
        // complete the square)

        p1 = new Point2D.Double(-10.0, -10.0);

        return result;
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
    }
}