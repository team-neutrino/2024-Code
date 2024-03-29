package frc.robot.commands.GamePieceCommands;

import frc.robot.subsystems.LEDSubsystem;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants.LEDConstants.States;

public class LEDDefaultCommand extends GamePieceCommand {

  private SwerveSubsystem m_swerveSubsystem;
  private LEDSubsystem m_LEDSubsystem;

  public LEDDefaultCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
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
    if (m_shooterSubsystem.approveShoot() && m_armSubsystem.getInPosition() && m_intakeSubsystem.isNoteReady()) {
      m_LEDSubsystem.setToGreen();
    } else if (m_intakeSubsystem.isNoteReady()) {
      m_LEDSubsystem.setToCyan();
    } else if (m_swerveSubsystem
        .getCommandState() == States.PATHFINDING) {
      m_LEDSubsystem.setToYellow();
    } else if (m_swerveSubsystem
        .getCommandState() == (States.AUTOALIGN)) {
      m_LEDSubsystem.setToBlue();
    } else if (m_intakeSubsystem.isNoteTooFar()) {
      m_LEDSubsystem.setToPurple();
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