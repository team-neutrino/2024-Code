package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutonShootManualCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ArmSubsystem m_armSubsystem;
    private double m_angle;
    private double m_rpm;
    double i = 0;

    public AutonShootManualCommand(double p_angle, double p_rpm) {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        m_armSubsystem = SubsystemContainer.armSubsystem;
        m_angle = p_angle;
        m_rpm = p_rpm;

        addRequirements(m_shooterSubsystem, m_armSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
            m_intakeSubsystem.runIndexShoot();
        } else {
            m_intakeSubsystem.stopIndex();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (!m_intakeSubsystem.m_intakeBeam) {
            i++;
        } else {
            i = 0;
        }
        if (i >= 20) {
            return true;
        }

        return false;
    }
}