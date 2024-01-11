package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
//import frc.robot.subsystems.SubsystemContainer;

public class LEDCommand extends Command {

    private LEDSubsystem m_LEDSubsystem;

public LEDCommand (LEDSubsystem p_LEDSubsystem){ //add subsystem container + driverstation info
    m_LEDSubsystem = p_LEDSubsystem;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_LEDSubsystem.setToGreen();;
  }

  @Override
  public void execute() {
    m_LEDSubsystem.setToGreen();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}



