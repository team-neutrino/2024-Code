package frc.robot.util;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.wrapper.Arm;
import frc.robot.subsystems.wrapper.Intake;
import frc.robot.subsystems.wrapper.NetworkTables;
import frc.robot.subsystems.wrapper.Shooter;

public class SubsystemContainer {
    public static CalculateAngle m_angleCalculate = new CalculateAngle();
    public static Alliance alliance = new Alliance();
    public static final NetworkTables sendables = new NetworkTables();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final Intake intakeSubsystem = new Intake();
    public static final Shooter shooterSubsystem = new Shooter();
    public static final Arm armSubsystem = new Arm(m_angleCalculate);
    public static CalculateP calculateP = new CalculateP();
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
}
