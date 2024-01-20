package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.SubsystemContainer;

public class LEDIntakeCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;

  public LEDIntakeCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    if (m_LEDSubsystem.getBeamBreak() == false) {
        m_LEDSubsystem.blinkBlue();
    }     
}

  @Override
  public void execute() {
    if (m_LEDSubsystem.getBeamBreak() == false) {
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
