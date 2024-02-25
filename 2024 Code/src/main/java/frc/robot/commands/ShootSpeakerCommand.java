package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeakerCommand extends Command {

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;

    public ShootSpeakerCommand() {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;

        addRequirements(m_shooterSubsystem, m_intakeSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setTargetRPM(3000);
        m_intakeSubsystem.indexApprove(m_shooterSubsystem.approveShoot());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}