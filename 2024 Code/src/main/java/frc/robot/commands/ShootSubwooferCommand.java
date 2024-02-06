package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.ArmSubsystem;

public class ShootSubwooferCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_indexSubsystem;
    private ArmSubsystem m_armSubsystem;

    public ShootSubwooferCommand() {
        m_shooterSubsystem = SubsystemContainer.ShooterSubsystem;
        m_indexSubsystem = SubsystemContainer.intakeSubsystem;
        m_armSubsystem = SubsystemContainer.armSubsystem;

        addRequirements(m_shooterSubsystem, m_indexSubsystem, m_armSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.armPID(Constants.ArmConstants.INTAKE_LIMIT);
        m_shooterSubsystem.setTargetRPM(ShooterSpeeds.SUBWOOFER_SPEED);
        if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
            m_indexSubsystem.runIndex();
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