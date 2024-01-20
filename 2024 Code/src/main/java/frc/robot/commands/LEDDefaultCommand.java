package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class LEDDefaultCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;

  public LEDDefaultCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    m_IntakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_LEDSubsystem, m_IntakeSubsystem);
  }

  @Override
  public void initialize() {
    m_LEDSubsystem.setToOrange();
    if (!m_IntakeSubsystem.getBeamBreak()) {
      m_LEDSubsystem.setToGreen();
    } else if (m_IntakeSubsystem.getBeamBreak()) {
      m_LEDSubsystem.setToOrange();
    }
  }

  @Override
  public void execute() {
    // System.out.println(m_IntakeSubsystem.getBeamBreak());
    m_LEDSubsystem.setToOrange();
    System.out.println("orange");
    if (!m_IntakeSubsystem.getBeamBreak()) {
      m_LEDSubsystem.setToGreen();
      System.out.println("green");
    } else if (m_IntakeSubsystem.getBeamBreak()) {
      m_LEDSubsystem.setToOrange();
      System.out.println("orange again");
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
