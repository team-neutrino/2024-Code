package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants.ShooterSpeeds;;;

public class ShooterDefaultCommand extends GamePieceCommand {
    double initialSpeed = 0;

    public ShooterDefaultCommand() {
    }

    public void initialize() {
        initialSpeed = m_shooterSubsystem.getShooterRPM();

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