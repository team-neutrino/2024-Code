package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeDefaultCommand extends Command {
    IntakeSubsystem m_intakeSubsystem;

    public IntakeDefaultCommand() {
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_intakeSubsystem.hasNote() && !m_intakeSubsystem.isNoteReady()) {
            m_intakeSubsystem.smartIntake();
        } else {
            m_intakeSubsystem.defaultIntake();
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
