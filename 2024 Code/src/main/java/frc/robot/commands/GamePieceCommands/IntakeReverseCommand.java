package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants.ArmConstants;

public class IntakeReverseCommand extends GamePieceCommand {
  public IntakeReverseCommand() {
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(700);
    m_armSubsystem.defaultArm();
    if (m_armSubsystem.getInPosition()) {
      m_intakeSubsystem.runIntakeReverse();
      m_intakeSubsystem.runIndexReverse();
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
