package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;

public interface ClimberIO {
    double getOutputCurrent = 0;

    public void setLeftSpeed(double speed);

    public void setRightSpeed(double speed);

    public double getLeftCurrent();

    public double getRightCurrent();

    public double getLeftMotorPosition();

    public double getRightMotorPosition();

    public boolean isLeftRetracted();

    public boolean isRightRetracted();

    public boolean isLeftSideStalling();

    public boolean isRightSideStalling();

    public RelativeEncoder getLeftEncoder();

    public RelativeEncoder getRightEncoder();

    public boolean isLeftExtended();

    public boolean isRightExtended();

    public void periodicUpdate();

}
