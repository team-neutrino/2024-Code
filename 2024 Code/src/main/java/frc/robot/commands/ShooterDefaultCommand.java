package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShooterDefaultCommand extends Command {
    double currentSpeed = 0;
    ShooterSubsystem m_shooterSubsystem;

    public ShooterDefaultCommand() {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
        currentSpeed = m_shooterSubsystem.getShooterRPM();

        if (currentSpeed == 0) {
            currentSpeed = Constants.ShooterSpeeds.INITIAL_SHOOTER_SPEED;
        }
    }

    @Override
    public void execute() {
        if (currentSpeed > ShooterSpeeds.INITIAL_SHOOTER_SPEED) {
            currentSpeed -= 10;
        }

        m_shooterSubsystem.setTargetRPM(currentSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}