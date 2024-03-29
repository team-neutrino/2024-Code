
package frc.robot.commands.GamePieceCommands;

public class AutonFeederCommand extends GamePieceCommand {

  public AutonFeederCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndexShoot();
    } else {
      m_intakeSubsystem.stopIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
