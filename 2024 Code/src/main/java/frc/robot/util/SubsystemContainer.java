package frc.robot.util;

import frc.robot.Constants.PWMConstants;
import frc.robot.subsystems.*;

public class SubsystemContainer {
    public static final CalculateAngle m_angleCalculate = new CalculateAngle();
    public static final Alliance alliance = new Alliance();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static final ArmSubsystem armSubsystem = new ArmSubsystem();
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem(PWMConstants.LED1);
}
