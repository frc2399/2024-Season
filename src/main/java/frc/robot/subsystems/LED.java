package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.Indexer;

public class LED extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(15);
    int m_rainbowFirstPixelHue = 0;
    //int m_rainbowLastPixelHue = 10;
    boolean isAutonomous = true;
    private Indexer indexer;

    public LED(Indexer indexer) {
        this.indexer = indexer;
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void rainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
          }
          // Increase by to make the rainbow "move"
          m_rainbowFirstPixelHue += 3;
          // Check bounds
          m_rainbowFirstPixelHue %= 180;
    }
   
    public void periodic() {
        if (isAutonomous) {
            rainbow();
        } 
        m_led.setData(m_ledBuffer);
    }
}