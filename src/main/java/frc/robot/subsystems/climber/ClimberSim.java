package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;

public class ClimberSim implements ClimberIO {

    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder;
    Debouncer leftDebouncer;
    Debouncer rightDebouncer;
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    @Override
    public void setLeftSpeed(double speed) {

    }

    @Override
    public void setRightSpeed(double speed) {

    }

    @Override
    public boolean isLeftExtended() {
        return true;
    }

    @Override
    public boolean isRightExtended() {
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
    public boolean isLeftSideStalling() {
        return true;
    }

    @Override
    public boolean isRightSideStalling() {
        return true;
    }

    @Override
    public double getLeftCurrent() {
        return 0.0;
    }

    @Override
    public double getRightCurrent() {
        return 0.0;
    }

    @Override
    public double getLeftMotorPosition() {

        return 0;
    }

    @Override
    public double getRightMotorPosition() {

        return 0;
    }

    @Override
    public RelativeEncoder getLeftEncoder() {

        return leftEncoder;
    }

    @Override
    public RelativeEncoder getRightEncoder() {

        return rightEncoder;
    }

    @Override
    public void periodicUpdate() {

    }

}
