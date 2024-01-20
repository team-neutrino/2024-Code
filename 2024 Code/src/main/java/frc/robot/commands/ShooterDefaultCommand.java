package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefaultCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public ShooterDefaultCommand() {
        addRequirements(SubsystemContainer.ShooterSubsystem);
    }


    public void initialize() {
    }
    
    @Override
    public void execute() {
        SubsystemContainer.ShooterSubsystem.setTargetRPM(1000);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}