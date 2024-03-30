package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultCommand extends Command {

  private LEDSubsystem m_LEDSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  public LEDDefaultCommand() {
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;

    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_LEDSubsystem.setToOrange();
  }

  @Override
  public void execute() {
    if (m_shooterSubsystem.approveShoot() && m_armSubsystem.getInPosition() && m_intakeSubsystem.isNoteReady()) {
      m_LEDSubsystem.setToGreen();
    } else if (m_intakeSubsystem.isNoteReady()) {
      m_LEDSubsystem.setToCyan();
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