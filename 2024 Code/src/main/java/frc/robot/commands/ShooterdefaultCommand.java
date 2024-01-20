package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterDefaultCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem m_shooterSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterDefaultCommand() {
    m_shooterSubsystem = SubsystemContainer.ShooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(4000);
  }
}
