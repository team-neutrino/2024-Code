package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.ArmSubsystem;

public class ShootManualCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_indexSubsystem;
    private ArmSubsystem m_armSubsystem;
    private double m_angle;
    private double m_rpm;

    public ShootManualCommand(double p_angle, double p_rpm) {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        m_indexSubsystem = SubsystemContainer.intakeSubsystem;
        m_armSubsystem = SubsystemContainer.armSubsystem;
        m_angle = p_angle;
        m_rpm = p_rpm;

        addRequirements(m_shooterSubsystem, m_indexSubsystem, m_armSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
            m_indexSubsystem.runIndexShoot();
        } else {
            m_indexSubsystem.stopIndex();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}