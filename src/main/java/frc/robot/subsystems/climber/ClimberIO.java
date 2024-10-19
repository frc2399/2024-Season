package frc.robot.subsystems.climber;

public interface ClimberIO {
    double getOutputCurrent = 0;

    public void setLeftSpeed(double speed);

    public void setRightSpeed(double speed);

    public boolean isLeftExtended();

    public boolean isRightExtended();

    public void setLeftHeight(double height);

    public void setRightHeight(double height);

    public boolean isLeftRetracted();

    public boolean isRightRetracted();

    public double getLeftEncoderPosition();

    public double getLeftCurrent();

    public double getRightCurrent();

    public double getRightEncoderPosition();

    public boolean isLeftSideStalling();

    public boolean isRightSideStalling();

    public void periodicUpdate();
}
