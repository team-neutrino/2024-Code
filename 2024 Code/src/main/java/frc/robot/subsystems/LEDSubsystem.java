package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PWMConstants;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_addressableLEDLeft;
    private AddressableLED m_addressableLEDRight;
    private AddressableLEDBuffer m_LEDBufferLeft;
    private AddressableLEDBuffer m_LEDBufferRight;
    private Timer timer = new Timer();

    public LEDSubsystem() {
        m_addressableLEDLeft = new AddressableLED(PWMConstants.LED_Left);
        m_addressableLEDRight = new AddressableLED(PWMConstants.LED_Right);
        m_LEDBufferLeft = new AddressableLEDBuffer(Constants.LEDConstants.LEDBufferLen);
        m_LEDBufferRight = new AddressableLEDBuffer(m_LEDBufferRight.getLength());
        m_addressableLEDLeft.setLength(m_LEDBufferLeft.getLength());
        m_addressableLEDLeft.setData(m_LEDBufferLeft);
        m_addressableLEDLeft.start();
        m_addressableLEDRight.setLength(m_LEDBufferRight.getLength());
        m_addressableLEDRight.setData(m_LEDBufferRight);
        m_addressableLEDRight.start();
        setToOrange();
        timer.start();
    }

    private void setToColor(int r, int g, int b) {
        for (int i = 0; i < m_LEDBufferLeft.getLength(); i++) {
            m_LEDBufferLeft.setRGB(i, r, g, b);
            m_LEDBufferRight.setRGB(i, r, g, b);
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

    // false = broken
    public void sarahStrobe() {
        double timeConst = Math.PI;
        int r = (int) Math.round(126 * Math.cos(timeConst / 4 * timer.get()) + 126);
        int g = (int) Math.round(126 * Math.cos(timeConst / 8 * timer.get()) + 126);
        int b = (int) Math.round(126 * Math.sin(timeConst / 2 * timer.get()) + 126);
        for (int i = 0; i < m_LEDBufferLeft.getLength(); ++i)
            setToColor(r, g, b);
    }

    public void periodic() {
        m_addressableLEDLeft.setData(m_LEDBufferLeft);
    }

}
