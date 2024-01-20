package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IndexDefaultCommand extends Command {

    private IntakeSubsystem m_indexSubsystem;

    public IndexDefaultCommand() {
        m_indexSubsystem = SubsystemContainer.intakeSubsystem;
        addRequirements(m_indexSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_indexSubsystem.getBeamBreak()) {
            m_indexSubsystem.runIndex();
        } else {
            m_indexSubsystem.stopIndex();
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
