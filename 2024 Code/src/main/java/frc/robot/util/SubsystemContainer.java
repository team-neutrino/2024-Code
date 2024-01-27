package frc.robot.util;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.simulation.ArmSimulation;
import frc.robot.subsystems.simulation.PIDChangerSimulation;
import frc.robot.subsystems.simulation.Shooter;

public class SubsystemContainer {
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static final Shooter ShooterSubsystem = new Shooter();
    public static final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    public static final ArmSimulation armSubsystem = new ArmSimulation();
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
}
