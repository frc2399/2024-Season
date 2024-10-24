package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void setMotor(double speed);

    public double getEncoderSpeed();

    public void periodicUpdate();

}
