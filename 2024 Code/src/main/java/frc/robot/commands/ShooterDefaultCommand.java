package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShooterDefaultCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;

    public ShooterDefaultCommand() {
        m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
        m_shooterSubsystem.setTargetRPM(4000); // ShooterSpeeds.INITIAL_SHOOTER_SPEED
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