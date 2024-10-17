package frc.robot.subsystems.Indexer;

public interface IndexerIO {
    public void setMotor(double indexerVelocity);

    public double getCurrent();

    public double getVelocity();

    public double getEncoderPosition();

    public void setCurrentLimit(int current);

    public void periodicUpdate();

    public boolean getIsBeamBroken();

    public void setIsOverride();
}
