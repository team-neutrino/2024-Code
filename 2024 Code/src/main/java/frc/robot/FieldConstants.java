package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

import java.util.List;
import java.util.Map;

// All values for the pose3d are in inches and the values for the rotation3d are in radians!
public final class FieldConstants{
    public static final Map<Integer, Pose3d> tenAprilTags =
    Map.of(
        1, new Pose3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, (2*Math.PI)/3)),
        2, new Pose3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, (2*Math.PI)/3)),
        3, new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Math.PI)),
        4, new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Math.PI)),
        5, new Pose3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323), Units.inchesToMeters(53.38), new Rotation3d(0, 0, (3*Math.PI)/2)),
        6, new Pose3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323), Units.inchesToMeters(53.38), new Rotation3d(0, 0, (3*Math.PI)/2)),
        7, new Pose3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d()),
        8, new Pose3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d()),
        9, new Pose3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Math.PI/3)),
        10, new Pose3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.58), new Rotation3d(0, 0, Math.PI/3))
    );
    public static final Map<Integer, Pose3d> sixteenAprilTags = 
    Map.of(
        11, new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52), new Rotation3d(0, 0, (5*Math.PI)/3)),
        12, new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52), new Rotation3d(0, 0, Math.PI/3)),
        13, new Pose3d(Units.inchesToMeters(441.73), Units.inchesToMeters(161.62), Units.inchesToMeters(52), new Rotation3d(0, 0, Math.PI)),
        14, new Pose3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52), new Rotation3d()),
        15, new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52), new Rotation3d(0, 0, (2*Math.PI)/3)),
        16, new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52), new Rotation3d(0, 0, (4*Math.PI)/3))
    );
    //FORMAT: M, C, X1, X2 (X, Z1, Z2 if null)
    public static final Double[][] bannedRegions = {
        {null, 2.423414, -1.557274, 1.557274},
        {null, -2.423414, -1.557274, 1.557274},
        {-0.55635208711, 2.90554543682, 2.423414, 5.222494},
        {0.55635208711, 2.90554543682, -5.222494, 2.423414},
        {0.55635208711, -2.90554543682, 2.423414, 5.222494},
        {-0.55635208711, -2.90554543682, -5.222494, 2.423414}
    };

    public static final Double[] autoAlignPoint = {1.0, 1.0, 1.0};
}