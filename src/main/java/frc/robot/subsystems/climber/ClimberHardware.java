package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberHardware implements ClimberIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final Debouncer leftDebouncer;
    private final Debouncer rightDebouncer;

    public ClimberHardware() {

        leftMotor = new CANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftDebouncer = new Debouncer(0.025);
        rightDebouncer = new Debouncer(0.025);

    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(speed);
        if ((isLeftRetracted() && speed < 0) || ((isLeftExtended() && speed > 0))
                || (isLeftSideStalling() && !isRightSideStalling() && speed < 0)) {
            leftMotor.set(0);
        } else if (leftEncoder.getPosition() < 0.10 && speed < 0) {
            leftMotor.set(-0.15);
        } else {
            leftMotor.set(speed);
        }
    }

    public void setRightSpeed(double speed) {
        rightMotor.set(speed);
        if ((isRightRetracted() && speed < 0) || ((isRightExtended() && speed > 0))
                || (isRightSideStalling() && !isLeftSideStalling() && speed < 0)) {
            rightMotor.set(0);
        } else if (rightEncoder.getPosition() < 0.10 && speed < 0) {
            rightMotor.set(-0.15);
        } else {
            rightMotor.set(speed);
        }
    }

    public double getLeftMotorPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightMotorPosition() {
        return rightMotor.getEncoder().getPosition();
    }

    public void stopMotors() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    @Override
    public double getLeftCurrent() {
        return leftMotor.getOutputCurrent();
    }

    @Override
    public double getRightCurrent() {

        return rightMotor.getOutputCurrent();
    }

    @Override
    public void periodicUpdate() {

    }

    @Override
    public boolean isLeftExtended() {
        return (leftEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isRightExtended() {
        return (rightEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    @Override
    public boolean isLeftRetracted() {

        return (leftEncoder.getPosition() < ClimberConstants.MIN_HEIGHT + 0.1);
    }

    @Override

    public RelativeEncoder getLeftEncoder() {
        return leftMotor.getEncoder();
    }

    @Override

    public RelativeEncoder getRightEncoder() {
        return rightMotor.getEncoder();
    }

    @Override
    public boolean isRightRetracted() {
        return (leftEncoder.getPosition() < ClimberConstants.MIN_HEIGHT + .01);

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

}