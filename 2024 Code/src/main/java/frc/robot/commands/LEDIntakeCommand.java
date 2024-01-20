package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class LEDIntakeCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;

  public LEDIntakeCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    m_IntakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_LEDSubsystem, m_IntakeSubsystem);
  }

  @Override
  public void initialize() {
    if (m_IntakeSubsystem.getBeamBreak() == false) {
      m_LEDSubsystem.blinkBlue();
    }
  }

  @Override
  public void execute() {
    if (m_IntakeSubsystem.getBeamBreak() == false) {
      m_LEDSubsystem.blinkBlue();
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
