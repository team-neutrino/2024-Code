package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeDefaultCommand extends Command {
    Timer timer;
    IntakeSubsystem m_intakeSubsystem;

    public IntakeDefaultCommand() {
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        timer = new Timer();
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (!m_intakeSubsystem.jitterComplete()) {
            timer.restart();
        }
    }

    @Override
    public void execute() {
        if (!timer.hasElapsed(2)) {
            m_intakeSubsystem.runIndexJitter();
        } else {
            timer.stop();

            if (!m_intakeSubsystem.isNoteReady()) {
                m_intakeSubsystem.runIndexFeed();
            } else {
                m_intakeSubsystem.defaultIntake();
            }
        }
    }

    /**
     * if (!timer.hasElapsed(2)) {
     * m_intakeSubsystem.runIndexJitter();
     * } else {
     * timer.stop();
     * 
     * if (!m_intakeSubsystem.isBeamBrokenIndex()) {
     * m_intakeSubsystem.runIndexFeed();
     * } else if (m_intakeSubsystem.hasNote() && !m_intakeSubsystem.isNoteReady()) {
     * m_intakeSubsystem.smartIntake();
     * } else {
     * m_intakeSubsystem.defaultIntake();
     * }
     * }
     */
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
