
package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberPlacebo implements ClimberIO {

    public void setLeftSpeed(double speed) {

    }

    @Override
    public void setRightSpeed(double speed) {

    }

    @Override
    public double getLeftCurrent() {
        return ClimberIO.getOutputCurrent;
    }

    @Override
    public double getRightCurrent() {
        return ClimberIO.getOutputCurrent;
    }

    public void periodicUpdate() {

    }

}