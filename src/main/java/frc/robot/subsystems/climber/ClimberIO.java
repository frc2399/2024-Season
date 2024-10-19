package frc.robot.subsystems.climber;

public interface ClimberIO {
    double getOutputCurrent = 0;

    public void setLeftSpeed(double speed);

    public void setRightSpeed(double speed);

    public double getLeftCurrent();

    public double getRightCurrent();

    public void periodicUpdate();

}
