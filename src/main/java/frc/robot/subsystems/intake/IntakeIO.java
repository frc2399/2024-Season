package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void setMotor(double speed);

    public double getLeftEncoderSpeed();

    public double getRightEncoderSpeed();

    public double getLeftEncoderPosition();

    public double getRightEncoderPosition();

    public void setLeftCurrentLimit(int current);

    public void setRightCurrentLimit(int current);

    public void periodicUpdate();
}
