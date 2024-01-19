package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeCommand extends Command {

  private IntakeSubsystem m_intakeSubsystem;

  public IntakeCommand() {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_intakeSubsystem.runIntake();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

