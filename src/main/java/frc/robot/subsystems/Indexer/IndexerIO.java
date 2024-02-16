package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.DigitalInput;

public interface IndexerIO {
    public void setMotor(double speed);
    public void setSpeed(double speed);
    public double getCurrent();
    public double getEncoderSpeed();
    public double getEncoderPosition();
    public void setCurrentLimit(int current);
    public void periodicUpdate();
    public void setIsIntooked(boolean intooked);
    public boolean getIsBeamBroken();
    public void setIsOverride(boolean override);
}
