package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;

public class LED extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(16);
    int m_rainbowFirstPixelHue = 97;
    int m_rainbowLastPixelHue = 154;
    private Vision vision;

    public LED(Vision vision) {
        this.vision = vision;
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void rainbow() {
        // for every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (int) (m_rainbowFirstPixelHue + (i * (155 - 97) / m_ledBuffer.getLength())) % (155 - 97);
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        if (m_rainbowFirstPixelHue == (m_rainbowLastPixelHue + 1)) {
            m_rainbowFirstPixelHue = 97;
        }
    }

    public void visionLED() {
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
        //turns off if nothing aligned
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                 m_ledBuffer.setHSV(i,0,0,0);
            }
        }
    }
    public void periodic() {
        //visionLED();
        m_led.setData(m_ledBuffer);
        if (RobotBase.isReal()) {
            if (Intake.isIntooked) {
                m_ledBuffer.setRGB(0, 112, 243, 121);
            // } else if (climber.isInClimberMode()) {
            //     m_ledBuffer.setRGB(0, 0, 100, 255);
            } else if (RobotBase.isSimulation()) {
                rainbow();
            }
        }
    }
}