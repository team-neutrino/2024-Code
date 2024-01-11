package frc.robot;
import frc.robot.subsystems.LEDSubsystem;

public class SubsystemContainer {
    private LEDSubsystem m_LEDSubsystem;
}

public SubsystemContainer(
    LEDSubsystem p_LEDSubsystem
) {
    m_LEDSubsystem = p_LEDSubsystem;
}

public LEDSubsystem getLEDSubsystem() {
    return m_LEDSubsystem;

}