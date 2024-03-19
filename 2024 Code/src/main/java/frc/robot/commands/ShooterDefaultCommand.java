package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShooterDefaultCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;
    double initialSpeed = 0;

    public ShooterDefaultCommand() {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
        initialSpeed = m_shooterSubsystem.getShooterRpm1();

        if (initialSpeed == 0) {
            initialSpeed = 700;
        }
    }

    @Override
    public void execute() {
        if (initialSpeed > ShooterSpeeds.INITIAL_SHOOTER_SPEED) {
            initialSpeed -= 10;
        }

        m_shooterSubsystem.setTargetRPM(initialSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}