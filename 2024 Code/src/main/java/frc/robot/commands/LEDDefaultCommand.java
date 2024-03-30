package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants.States;

public class LEDDefaultCommand extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private LEDSubsystem m_LEDSubsystem;

  public LEDDefaultCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;

    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_LEDSubsystem.setToOrange();
  }

  @Override
  public void execute() {
    if (SubsystemContainer.shooterSubsystem.approveShoot() && SubsystemContainer.armSubsystem.getInPosition()
        && SubsystemContainer.intakeSubsystem.isNoteReady()) {
      m_LEDSubsystem.setToGreen();
    } else if (SubsystemContainer.intakeSubsystem.isNoteReady()) {
      m_LEDSubsystem.setToCyan();
    } else if (m_swerveSubsystem
        .getCommandState() == States.PATHFINDING) {
      m_LEDSubsystem.setToYellow();
    } else if (m_swerveSubsystem
        .getCommandState() == (States.AUTOALIGN)) {
      m_LEDSubsystem.setToBlue();
    } else if (SubsystemContainer.intakeSubsystem.isNoteTooFar()) {
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