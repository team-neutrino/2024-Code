package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants.LEDConstants.States;

public class LEDDefaultCommand extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private LEDSubsystem m_LEDSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;

  public LEDDefaultCommand() {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
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
    if (m_swerveSubsystem == null) {
      return;
    }
    if (m_IntakeSubsystem.runIndexFeedCheck()) {
      m_LEDSubsystem.setToGreen();
    } else if (m_swerveSubsystem
        .getCommandState() == States.PATHFINDING) {
      m_LEDSubsystem.setToYellow();
    } else if (m_swerveSubsystem
        .getCommandState() == (States.AUTOALIGN)) {
      m_LEDSubsystem.setToBlue();
    } else if (m_IntakeSubsystem.isBeamBrokenIntake() && m_IntakeSubsystem.isBeamBrokenIndex()) {
      m_LEDSubsystem.setToRed();
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
