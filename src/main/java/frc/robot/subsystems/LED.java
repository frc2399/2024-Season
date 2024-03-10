package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;

public class LED extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(16);
    int m_rainbowFirstPixelHue = 17;
    int m_rainbowLastPixelHue = 100;
    private Climber climber;

    public LED(Climber climber) {
        this.climber = climber;
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void rainbow() {
        // for every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (int) (m_rainbowFirstPixelHue + (i * (175 - 17) / m_ledBuffer.getLength())) % (175 - 17);
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        m_rainbowFirstPixelHue %= 175;
    }

    @Override
    public void periodic() {
        rainbow();
        if (RobotBase.isReal()) {
            // if (Intake.isIntooked) {
            //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            //         m_ledBuffer.setRGB(i, 112, 243, 121);
            //     }
            // } else 
            // if (climber.isInClimberMode()) {
            //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            //         m_ledBuffer.setRGB(i, 0, 100, 255);
            //     }
            // } else {
                rainbow();
            // }
        }
        m_led.setData(m_ledBuffer);
    }
}