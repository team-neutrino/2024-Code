package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import java.util.Map;

// All values for the pose3d are in inches and the values for the rotation3d are in radians!
public final class FieldConstants2025 {

        private static double degreesToRadians(double angle) {
                return angle * Math.PI / 180;
        }

        public static final Map<Integer, Pose3d> tenAprilTags = Map.of(
                        1,
                        new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.8),
                                        Units.inchesToMeters(58.50),
                                        new Rotation3d(degreesToRadians(126), 0, 0)),
                        2,
                        new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20),
                                        Units.inchesToMeters(58.5),
                                        new Rotation3d(degreesToRadians(234), 0, 0)),
                        3,
                        new Pose3d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15),
                                        Units.inchesToMeters(51.25),
                                        new Rotation3d(degreesToRadians(270), 0, 0)),
                        4,
                        new Pose3d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(0, 0, degreesToRadians(30))),
                        5,
                        new Pose3d(Units.inchesToMeters(365.20), Units.inchesToMeters(75.39),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(0, 0, degreesToRadians(30))),
                        6,
                        new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(300), 0, 0)),
                        7,
                        new Pose3d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(0, 0, 0)),
                        8,
                        new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(60), 0, 0)),
                        9,
                        new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(120), 0, 0)),
                        10,
                        new Pose3d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.5),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(180), 0, 0)));

        public static final Map<Integer, Pose3d> twentyAprilTags = Map.of(
                        11,
                        new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(240), 0, 0)),
                        12,
                        new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8),
                                        Units.inchesToMeters(58.5),
                                        new Rotation3d(degreesToRadians(54), 0, 0)),
                        13,
                        new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8),
                                        Units.inchesToMeters(58.5),
                                        new Rotation3d(degreesToRadians(306), 0, 0)),
                        14,
                        new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(degreesToRadians(180), 0, 0)),
                        15,
                        new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(degreesToRadians(180), 0, 0)),
                        16,
                        new Pose3d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(degreesToRadians(90), 0, 0)),
                        17,
                        new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(240), 0, 0)),
                        18,
                        new Pose3d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(180), 0, 0)),
                        19,
                        new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(120), 0, 0)),
                        20,
                        new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(60), 0, 0)));

        public static final Map<Integer, Pose3d> twentyTwoAprilTags = Map.of(
                        21,
                        new Pose3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(0), 0, 0)),
                        22,
                        new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(degreesToRadians(300), 0, 0)));

        public static final Double COMMUNITYBOUNDARY = 5.0; // 2.423414;
}