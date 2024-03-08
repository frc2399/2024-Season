package frc.robot.subsystems.Indexer;

public interface IndexerIO {
    public void setMotor(double speed);

    public double getCurrent();

    public double getEncoderSpeed();

    public double getEncoderPosition();

    public void setCurrentLimit(int current);

    public void periodicUpdate();

    public void setIsIntooked(boolean intooked);

    public boolean getIsBeamBroken();

    public boolean getIsIntooked();

    public boolean isStalling();
}
