package frc.robot.subsystems.Indexer;

public interface IndexerIO {
    public void setMotor(double speed);
    public void setSpeed(double speed);
    public double getCurrent();
    public double getEncoderSpeed();
    public double getEncoderPosition();
    public void setCurrentLimit(int current);
    public void periodicUpdate();
  
}
