// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.awt.geom.Point2D;

/** Add your docs here. */
public class PolarCoord extends Point2D.Double {

    public PolarCoord(double r, double theta) throws IllegalArgumentException {
        super(r, theta);

        if (r < 0) {
            throw new IllegalArgumentException("r cannot be negative");
        }

    }

    public PolarCoord() {
        super(0, 0);
    }

    public double getRadius() {
        return this.getX();
    }

    public double getTheta() {
        return this.getY();
    }

    // @Override
    public double distance(PolarCoord other) {
        // if (this.getClass() == point.getClass()) {
        // throw new IllegalArgumentException("argument must be a polar coord");
        // }

        // // safely convert to PolarCoord object
        // PolarCoord other = (PolarCoord) point;

        return Math.sqrt(Math.pow(this.getRadius(), 2) + Math.pow(other.getRadius(), 2)
                - 2 * this.getRadius() * other.getRadius() * Math.cos(other.getTheta() - this.getTheta()));
    }

    @Override
    public void setLocation(double r, double theta) {
        super.setLocation(r, theta);
    }

}
