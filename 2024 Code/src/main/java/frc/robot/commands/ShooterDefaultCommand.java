package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShooterDefaultCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;

    public ShooterDefaultCommand() {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setTargetRPM(700);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}