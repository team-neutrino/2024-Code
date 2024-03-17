package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ShootManualCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ArmSubsystem m_armSubsystem;
    private double m_angle;
    private double m_rpm;
    private CommandXboxController m_Controller;
    double i = 0;

    public ShootManualCommand(double p_angle, double p_rpm, CommandXboxController p_Controller) {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        m_armSubsystem = SubsystemContainer.armSubsystem;
        m_angle = p_angle;
        m_rpm = p_rpm;
        m_Controller = p_Controller;

        addRequirements(m_shooterSubsystem, m_armSubsystem, m_intakeSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        m_intakeSubsystem.runIndexFeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_Controller.getHID().getLeftBumper() && m_armSubsystem.getInPosition()
        && m_shooterSubsystem.approveShoot() && m_intakeSubsystem.isCentered();
    }
}