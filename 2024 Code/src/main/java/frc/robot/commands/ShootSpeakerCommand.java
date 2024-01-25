package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeakerCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_indexSubsystem;

    public ShootSpeakerCommand() {
        m_shooterSubsystem = SubsystemContainer.ShooterSubsystem;
        m_indexSubsystem = SubsystemContainer.intakeSubsystem;

        addRequirements(m_shooterSubsystem, m_indexSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setTargetRPM(1000);
        m_indexSubsystem.indexApprove(m_shooterSubsystem.approveShoot());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}