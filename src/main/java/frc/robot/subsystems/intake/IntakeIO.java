package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void setMotor(double intakeVelocity);

    public double getLeftVelocity();

    public double getRightVelocity();

    public double getLeftEncoderPosition();

    public double getRightEncoderPosition();

    public void setLeftCurrentLimit(int current);

    public void setRightCurrentLimit(int current);

    public void periodicUpdate();
}
