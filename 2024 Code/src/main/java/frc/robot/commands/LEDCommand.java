package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.SubsystemContainer;

public class LEDCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;

  public LEDCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_LEDSubsystem.setToGreen();
  }

  @Override
  public void execute() {
    m_LEDSubsystem.setToGreen();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
