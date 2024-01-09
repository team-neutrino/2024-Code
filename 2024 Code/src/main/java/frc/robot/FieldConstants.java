package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.List;
import java.util.Map;

// All values for the pose3d are in inches and the values for the rotation3d are in radians!
public final class FieldConstants{
    public static final Map<Integer, Pose3d> tenAprilTags =
    Map.of(
        1, new Pose3d(593.68, 9.68, 53.38, new Rotation3d(0, 0, (2*Math.PI)/3)),
        2, new Pose3d(637.21, 34.79, 53.38, new Rotation3d(0, 0, (2*Math.PI)/3)),
        3, new Pose3d(652.73, 196.17, 57.13, new Rotation3d(0, 0, Math.PI)),
        4, new Pose3d(652.73, 218.42, 57.13, new Rotation3d(0, 0, Math.PI)),
        5, new Pose3d(578.77, 323, 53.38, new Rotation3d(0, 0, (3*Math.PI)/2)),
        6, new Pose3d(72.5, 323, 53.38, new Rotation3d(0, 0, (3*Math.PI)/2)),
        7, new Pose3d(-1.5, 218.42, 57.13, new Rotation3d()),
        8, new Pose3d(-1.5, 196.17, 57.13, new Rotation3d()),
        9, new Pose3d(14.02, 34.79, 53.38, new Rotation3d(0, 0, Math.PI/3)),
        10, new Pose3d(57.54, 9.68, 53.58, new Rotation3d(0, 0, Math.PI/3))
    );
    public static final Map<Integer, Pose3d> sixteenAprilTags = 
    Map.of(
        11, new Pose3d(468.69, 146.19, 52, new Rotation3d(0, 0, (5*Math.PI)/3)),
        12, new Pose3d( 468.69, 177.10, 52, new Rotation3d(0, 0, Math.PI/3)),
        13, new Pose3d(441.73, 161.62, 52, new Rotation3d(0, 0, Math.PI)),
        14, new Pose3d(209.48, 161.62, 52, new Rotation3d()),
        15, new Pose3d(182.73, 177.10, 52, new Rotation3d(0, 0, (2*Math.PI)/3)),
        16, new Pose3d(182.73, 146.19, 52, new Rotation3d(0, 0, (4*Math.PI)/3))
    );
}