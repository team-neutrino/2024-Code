package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PWMConstants;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_addressableLED1;
    private AddressableLED m_addressableLED2;
    private AddressableLEDBuffer m_LEDBuffer;
    private Timer timer = new Timer();

    public LEDSubsystem() {
        m_addressableLED1 = new AddressableLED(PWMConstants.LED1);
        m_addressableLED2 = new AddressableLED(PWMConstants.LED2);
        m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.LEDBufferLen);
        m_addressableLED1.setLength(m_LEDBuffer.getLength());
        m_addressableLED2.setLength(m_LEDBuffer.getLength());
        m_addressableLED1.setData(m_LEDBuffer);
        m_addressableLED2.setData(m_LEDBuffer);
        m_addressableLED1.start();
        m_addressableLED2.start();
        setToOrange();
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

    public void setToBlue() {
        setToColor(0, 0, 255);
    }

    public void setToYellow() {
        setToColor(255, 255, 0);
    }

    public void setToRed() {
        setToColor(255, 0, 0);
    }

    public void setToPurple() {
        setToColor(255, 0, 255);
    }

    public void setToCyan() {
        setToColor(0, 255, 255);
    }

    // false = broken
    public void sarahStrobe() {
        double timeConst = Math.PI;
        int r = (int) Math.round(126 * Math.cos(timeConst / 4 * timer.get()) + 126);
        int g = (int) Math.round(126 * Math.cos(timeConst / 8 * timer.get()) + 126);
        int b = (int) Math.round(126 * Math.sin(timeConst / 2 * timer.get()) + 126);
        for (int i = 0; i < m_LEDBuffer.getLength(); ++i)
            setToColor(r, g, b);
    }

    public void periodic() {
        m_addressableLED1.setData(m_LEDBuffer);
        m_addressableLED2.setData(m_LEDBuffer);
    }

}
