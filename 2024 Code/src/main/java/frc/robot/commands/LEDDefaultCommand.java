package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.SubsystemContainer;

public class LEDDefaultCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;

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
    if (SubsystemContainer.swerveSubsystem == null) {
      return;
    }
    if (!m_IntakeSubsystem.getBeamBreak()) {
      m_LEDSubsystem.setToGreen();
    } else if (SubsystemContainer.swerveSubsystem
        .getCommandState() == frc.robot.Constants.LEDConstants.States.PATHFINDING) {
      m_LEDSubsystem.setToYellow();
    } else if (SubsystemContainer.swerveSubsystem.getCommandState()
        .equals(frc.robot.Constants.LEDConstants.States.AUTOALIGN)) {
      m_LEDSubsystem.setToBlue();
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
