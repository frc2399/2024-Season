package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.vision.Vision;

public class LED extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(16);
    int m_rainbowFirstPixelHue = 140;
    int m_rainbowLastPixelHue = 10;
    boolean isAutonomous = true;
    private Vision vision;
    private Indexer indexer;

    public LED(Vision vision, Indexer indexer) {
        this.vision = vision;
        this.indexer = indexer;
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void rainbow() {
        // for every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (int) (m_rainbowFirstPixelHue + (i * (50) / m_ledBuffer.getLength())) % (180);
            if (hue > m_rainbowLastPixelHue && hue < m_rainbowFirstPixelHue) {
                hue += 130;
            }
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by 1 to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        m_rainbowFirstPixelHue %= 180;
        // Check bounds
        if (m_rainbowFirstPixelHue == (m_rainbowLastPixelHue + 1)) {
            m_rainbowFirstPixelHue = 140;
        }
    }

    public void turnTeleop() {
        isAutonomous = false;
    }

    public void teleopLed() {
        //purple if driveTrainAligned AND arm aligned
        if (vision.isArmAligned() && vision.isDriveTrainAligned()) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 128, 102, 236);
            }
        //blue if just arm aligned
        } else if (vision.isArmAligned() && !vision.isDriveTrainAligned()) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 204, 255);
            }
        //pink if just drive aligned
        } else if (!vision.isArmAligned() && vision.isDriveTrainAligned()) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 255, 0, 216);
            }
        //turns green if note is intook
        } else if (indexer.isIntooked) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 7, 250, 20);
            }
        //turns off if nothing
        } else {
            rainbow();
        }
    }
    public void periodic() {
        if (isAutonomous) {
            rainbow();
        } else {
            teleopLed();
        }
        m_led.setData(m_ledBuffer);
    }
}