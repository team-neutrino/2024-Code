package frc.robot.util;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.simulation.Climb;
import frc.robot.subsystems.simulation.IntakeSimulation;
import frc.robot.subsystems.simulation.ArmSimulation;
import frc.robot.subsystems.simulation.Shooter;
import frc.robot.subsystems.simulation.SwerveSim;

public class SubsystemContainer {
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSim swerveSubsystem = new SwerveSim();
    public static final IntakeSimulation intakeSubsystem = new IntakeSimulation();
    public static final Shooter ShooterSubsystem = new Shooter();
    public static final Climb climbSubsystem = new Climb();
    public static final ArmSimulation armSubsystem = new ArmSimulation();
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
}
