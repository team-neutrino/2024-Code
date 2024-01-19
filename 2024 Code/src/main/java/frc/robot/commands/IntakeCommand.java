package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DigitalConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeCommand extends Command {

  private IntakeSubsystem m_intakeSubsystem;

  private DigitalInput m_intakeDownSensor = new DigitalInput(DigitalConstants.INTAKE_MOTOR_BEAMBREAK);

  public IntakeCommand() {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // when beam break is tripped, .get() returns false
    // motor should run while the beam break is NOT tripped
    if (m_intakeDownSensor.get()) {
      m_intakeSubsystem.runIntake();
    } else {
      m_intakeSubsystem.stopIntake();
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
