package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;
import frc.robot.Constants.LEDConstants.States;

public class LEDDefaultCommand extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private LEDSubsystem m_LEDSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;
  private ShooterSubsystem m_ShooterSubsystem;
  private ArmSubsystem m_ArmSubsystem;

  public LEDDefaultCommand() {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_LEDSubsystem = SubsystemContainer.LEDSubsystem;
    m_IntakeSubsystem = SubsystemContainer.intakeSubsystem;
    m_ShooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_ArmSubsystem = SubsystemContainer.armSubsystem;
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
    if (m_IntakeSubsystem.isCentered()) {
      m_LEDSubsystem.setToCyan();
    } else if (m_swerveSubsystem
        .getCommandState() == States.PATHFINDING) {
      m_LEDSubsystem.setToYellow();
    } else if (m_swerveSubsystem
        .getCommandState() == (States.AUTOALIGN)) {
      m_LEDSubsystem.setToBlue();
    } else if (m_ShooterSubsystem.approveShoot() && m_ArmSubsystem.getInPosition() && m_IntakeSubsystem.isCentered()) {
      m_LEDSubsystem.setToGreen();
    } else if (m_IntakeSubsystem.tooFarNote()) {
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
