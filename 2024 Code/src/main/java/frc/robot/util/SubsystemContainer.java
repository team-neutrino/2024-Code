package frc.robot.util;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SubsystemContainer {
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

}
