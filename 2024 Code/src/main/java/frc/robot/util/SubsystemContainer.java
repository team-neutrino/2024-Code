package frc.robot.util;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.simulation.IntakeSimulation;
import frc.robot.subsystems.simulation.ArmSimulation;
import frc.robot.subsystems.simulation.Shooter;
import frc.robot.subsystems.simulation.SimulationOverview;

public class SubsystemContainer {

    public static final SimulationOverview simOverview = new SimulationOverview();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final IntakeSimulation intakeSubsystem = new IntakeSimulation();
    public static final Shooter shooterSubsystem = new Shooter();
    // public static final Climb climbSubsystem = new Climb();
    public static final ArmSimulation armSubsystem = new ArmSimulation();
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
}
