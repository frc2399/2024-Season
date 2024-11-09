
package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ClimberConstants;

public class ClimberPlacebo implements ClimberIO {

    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder;
    Debouncer leftDebouncer;
    Debouncer rightDebouncer;
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

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

    @Override
    public double getLeftMotorPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public boolean isRightExtended() {
        return true;
    }

    @Override
    public boolean isLeftExtended() {
        return true;
    }

    @Override
    public boolean isLeftRetracted() {
        return true;
    }

    @Override
    public boolean isRightRetracted() {
        return true;
    }

    @Override
    public RelativeEncoder getRightEncoder() {

        return rightEncoder;

    }

    @Override
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    @Override
    public boolean isLeftSideStalling() {
        return leftDebouncer
                .calculate(Math.abs(getLeftCurrent()) > Constants.ClimberConstants.CURRENT_THRESHOLD);
    }

    @Override
    public boolean isRightSideStalling() {
        return rightDebouncer
                .calculate(Math.abs(getRightCurrent()) > Constants.ClimberConstants.CURRENT_THRESHOLD);
    }

    public void periodicUpdate() {

    }

    @Override
    public double getRightMotorPosition() {
        return rightMotor.getEncoder().getPosition();

    }

}