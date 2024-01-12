package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.PWMConstants;
//import frc.robot.util.EnumConstants.LEDColor;
//import frc.robot.commands.LEDDefaultCommand;

public class LEDSubsystem extends SubsystemBase {
    public AddressableLED m_addressableLED;
    public AddressableLEDBuffer m_LEDBuffer;
    private Timer timer = new Timer();

    public LEDSubsystem() {
        m_addressableLED = new AddressableLED(9); // PWM port number 9
        m_LEDBuffer = new AddressableLEDBuffer(60);
        m_addressableLED.setLength(m_LEDBuffer.getLength());
        m_addressableLED.setData(m_LEDBuffer);
        m_addressableLED.start();
        setToOrange();
        // sarahStrobe();
        timer.start();
    }

    private void setToColor(int r, int g, int b) {
        for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
            m_LEDBuffer.setRGB(i, r, g, b);
        }
    }

    public void setToOrange() {
        setToColor(235, 20, 0);
    }

    public void setToGreen() {
        setToColor(0, 255, 0);
    }

    private void sarahStrobe() {
        double timeConst = Math.PI;
        int r = (int) Math.round(126 * Math.cos(timeConst / 4 * timer.get()) + 126);
        int g = (int) Math.round(126 * Math.cos(timeConst / 8 * timer.get()) + 126);
        int b = (int) Math.round(126 * Math.sin(timeConst / 2 * timer.get()) + 126);
        for (int i = 0; i < m_LEDBuffer.getLength(); ++i)
            setToColor(r, g, b);
    }

    public void periodic() {
        m_addressableLED.setData(m_LEDBuffer);
    }

}
