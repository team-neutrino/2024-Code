package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.SubsystemContainer;

public class LEDDefaultCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;
  private AutoAlignCommand m_AutoAlignCommand;
  private AutoAlignSequentialCommand m_AutoAlignSequentialCommand;

  // ask aneesh what the diff between AutoAlignCommand
  // and AutoAlignSequentialCommand is

  public LEDDefaultCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    m_IntakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_LEDSubsystem.setToOrange();
  }

  @Override
  public void execute() {
    if (!m_IntakeSubsystem.getBeamBreak()) {
      m_LEDSubsystem.setToGreen();
    } else if (!m_AutoAlignSequentialCommand.isFinished()) {
      m_LEDSubsystem.setToYellow();
    } else if (m_AutoAlignSequentialCommand.isFinished()) {
      m_LEDSubsystem.blueTimer();
    } else {
      m_LEDSubsystem.setToOrange();
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
