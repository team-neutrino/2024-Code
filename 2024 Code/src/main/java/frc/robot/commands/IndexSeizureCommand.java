package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IndexSeizureCommand extends Command {

    private IntakeSubsystem m_intakeSubsystem;

    public IndexSeizureCommand() {
        m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!m_intakeSubsystem.getBeamBreak()) {
            for (int i = 0; i < 250; i++) {
                if ((i / 10) % 2 == 0) {
                    m_intakeSubsystem.runIndex();
                    System.out.println("forward index");
                } else {
                    m_intakeSubsystem.runIndexReverse();
                    System.out.println("reverse index");
                }
            }
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
