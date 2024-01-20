package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeDefaultCommand extends Command {

    private IntakeSubsystem m_intakeSubsystem;

    public IntakeDefaultCommand() {
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // motor should run while the beam break is NOT tripped
        if (m_intakeSubsystem.getBeamBreak()) {
            System.out.println("motors should be running");
            m_intakeSubsystem.runIntake();
            m_intakeSubsystem.runIndex();
        } else {
            System.out.println("everything should be stopped");
            m_intakeSubsystem.stopIntake();
            m_intakeSubsystem.stopIndex();
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
