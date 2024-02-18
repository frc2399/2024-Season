package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LEDController;

public class LED extends SubsystemBase {
    LEDController red = new LEDController(Constants.LEDConstants.RED_CHANNEL);
    LEDController green = new LEDController(Constants.LEDConstants.GREEN_CHANNEL);
    LEDController blue = new LEDController(Constants.LEDConstants.BLUE_CHANNEL);
    LEDController white = new LEDController(Constants.LEDConstants.WHITE_CHANNEL);
    private Climber climber;

    public LED(Climber climber) {
        this.climber = climber;
    }

    public void setColor(int r, int g, int b, int w) {
        red.set(r);
        green.set(g);
        blue.set(b);
        white.set(w);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            if (Intake.isIntooked) {
                this.setColor(112, 243, 121, 0);
            } else if (RobotBase.isSimulation()) {
                this.setColor(255, 50, 200, 0);
            } else if (climber.isInClimberMode()) {
                this.setColor(0, 100, 255, 0);
            }
        }
    }

}