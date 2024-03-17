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
    int m_rainbowFirstPixelHue = 97;
    int m_rainbowLastPixelHue = 154;
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
            final var hue = (int) (m_rainbowFirstPixelHue + (i * (155 - 97) / m_ledBuffer.getLength())) % (180);
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by 1 to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        if (m_rainbowFirstPixelHue == (m_rainbowLastPixelHue + 1)) {
            m_rainbowFirstPixelHue = 97;
        }
    }

    public void turnTeleop() {
        isAutonomous = false;
    }

    public void teleopLed() {
        //purple if driveTrainAligned AND arm aligned
        if (vision.isArmAligned() && vision.isDriveTrainAligned()) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, 125, 255, 128);
            }
        //blue if just arm aligned
        } else if (vision.isArmAligned() && !vision.isDriveTrainAligned()) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, 97, 255, 128);
            }
        //pink if just drive aligned
        } else if (!vision.isArmAligned() && vision.isDriveTrainAligned()) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, 154, 255, 128);
            }
        //turns green if note is intook
        } else if (indexer.isIntooked) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, 80, 255, 128);
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