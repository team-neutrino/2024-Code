package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutonSingleShotCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ArmSubsystem m_armSubsystem;
    private double m_angle;
    private double m_rpm;
    // Use a timer like everywhere else
    double i = 0;

    public AutonSingleShotCommand(double p_angle, double p_rpm) {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        m_armSubsystem = SubsystemContainer.armSubsystem;
        m_angle = p_angle;
        m_rpm = p_rpm;

        addRequirements(m_shooterSubsystem);
        addRequirements(m_armSubsystem);
        addRequirements(m_intakeSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        // Rename getInPosition to isInPosition
        if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
            m_intakeSubsystem.runIndexShoot();
        } else {
            // Still suspicious
            m_intakeSubsystem.stopIndex();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Use a timer
        if (!m_intakeSubsystem.isBeamBrokenIntake()) {
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