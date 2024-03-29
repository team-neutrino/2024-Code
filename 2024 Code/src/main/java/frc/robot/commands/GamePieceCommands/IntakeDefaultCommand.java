package frc.robot.commands.GamePieceCommands;

public class IntakeDefaultCommand extends GamePieceCommand {
    public IntakeDefaultCommand() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_intakeSubsystem.stopIntake();
        m_intakeSubsystem.stopIndex();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
