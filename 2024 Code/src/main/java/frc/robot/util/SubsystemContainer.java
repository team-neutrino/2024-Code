package frc.robot.util;

import frc.robot.Constants.PWMConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.wrapper.NetworkTables;

public class SubsystemContainer {
    public static final CalculateAngle m_angleCalculate = new CalculateAngle();
    public static final Alliance alliance = new Alliance();
    public static final NetworkTables sendables = new NetworkTables();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final PhotonVision photonVision = new PhotonVision();
}
