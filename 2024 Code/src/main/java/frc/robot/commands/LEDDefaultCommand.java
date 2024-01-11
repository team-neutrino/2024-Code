package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDDefaultCommand extends Command {

    private LEDSubsystem m_LEDSubsystem;

public LEDDefaultCommand(LEDSubsystem p_LEDSubsystem){
    m_LEDSubsystem = p_LEDSubsystem;
    addRequirements(m_LEDSubsystem);
}

@Override
public void initialize() {
  m_LEDSubsystem.setToOrange();;
}

@Override
public void execute() {
  m_LEDSubsystem.setToOrange();
}

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
  return false;
}
}


