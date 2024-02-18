package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void setMotor(double speed);

    public double getCurrent();

    public double getEncoderSpeed();
    public void setCurrentLimit(int current);

    public void periodicUpdate();
    public void setSpeed(double speed);
}
