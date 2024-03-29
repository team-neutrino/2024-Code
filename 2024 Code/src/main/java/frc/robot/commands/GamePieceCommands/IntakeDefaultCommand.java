package frc.robot.commands.GamePieceCommands;

;;

public class IntakeDefaultCommand extends GamePieceCommand {
    public IntakeDefaultCommand() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // motor should run while the beam break is NOT tripped
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
