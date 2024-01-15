package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void setMotor(double speed);
    public double getCurrent();
    public double getEncoderSpeed();
    public double getEncoderPosition();
    public void setPosition(double position);
    public void setCurrentLimit(int current);
    public void periodicUpdate();
    public void setMotorWithPID(double speed);
}
