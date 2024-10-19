package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class ClimberHardware implements ClimberIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public ClimberHardware() {

        leftMotor = new CANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(speed);
    }

    public void setRightSpeed(double speed) {
        rightMotor.set(speed);
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getLeftCurrent'");
    }

    @Override
    public double getRightCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRightCurrent'");
    }

    @Override
    public void periodicUpdate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodicUpdate'");
    }

}