
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutonFeederCommand extends Command {

  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private ArmSubsystem m_ArmSubsystem;
  int i;

  public AutonFeederCommand() {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_ArmSubsystem = SubsystemContainer.armSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.stopIntake();
    if (m_ArmSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndexShoot();
    } else {
      m_intakeSubsystem.stopIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_intakeSubsystem.isBeamBrokenIntake()) {
      i++;
      if (i > 10) {
        return true;
      }
    }
    return false;
  }
}
