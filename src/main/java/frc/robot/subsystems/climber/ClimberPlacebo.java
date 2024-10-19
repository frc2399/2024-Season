
package frc.robot.subsystems.Climber;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberPlacebo implements ClimberIO {

    public void setLeftSpeed(double speed) {

    }

    @Override
    public void setLeftHeight(double height) {
    }

    @Override
    public void setRightHeight(double height) {

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

    public boolean isLeftExtended(){
        return
    }

    public boolean isRightExtended() {

    }

    public boolean isRightSideStalling() {
        return rightDebouncer
                .calculate(Math.abs(climberIO.getRightCurrent()) > Constants.ClimberConstants.CURRENT_THRESHOLD);
    }

    public boolean isRightRetracted() {
        return (rightEncoder.getPosition() < ClimberConstants.MIN_HEIGHT + .01);
    }
}