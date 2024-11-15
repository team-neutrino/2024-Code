package frc.robot.util;

import frc.robot.Constants.PWMConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.wrapper.Arm;
import frc.robot.subsystems.wrapper.Intake;
import frc.robot.subsystems.wrapper.NetworkTables;
import frc.robot.subsystems.wrapper.Shooter;

public class SubsystemContainer {
    public static final AntiInterpolationCalculation m_calculateMovingShot = new AntiInterpolationCalculation();
    public static final CalculateAngle m_angleCalculate = new CalculateAngle();
    public static final Alliance alliance = new Alliance();
    public static final NetworkTables sendables = new NetworkTables();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final Intake intakeSubsystem = new Intake();
    public static final Shooter shooterSubsystem = new Shooter();
    public static final Arm armSubsystem = new Arm(m_angleCalculate);
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem(PWMConstants.LED1);
    public static final CommandSwerveDrivetrain swerveSubsystem2 = TunerConstants.DriveTrain;
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final RobotInputListener inputListener = new RobotInputListener();
}
