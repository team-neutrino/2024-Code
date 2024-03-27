package frc.robot.util;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.wrapper.ArmSimulation;
import frc.robot.subsystems.wrapper.IntakeSimulation;
import frc.robot.subsystems.wrapper.NetworkTables;
import frc.robot.subsystems.wrapper.Shooter;
import frc.robot.subsystems.wrapper.SimulationOverview;

public class SubsystemContainer {
    public static CalculateAngle m_angleCalculate = new CalculateAngle();
    public static final SimulationOverview simOverview = new SimulationOverview();
    public static final NetworkTables sendables = new NetworkTables();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final IntakeSimulation intakeSubsystem = new IntakeSimulation();
    public static final Shooter shooterSubsystem = new Shooter();
    public static final ArmSimulation armSubsystem = new ArmSimulation(m_angleCalculate);
    public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
}
