package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefaultCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;

    public ShooterDefaultCommand() {
        m_shooterSubsystem = SubsystemContainer.ShooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
        m_shooterSubsystem.stopShooter();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}