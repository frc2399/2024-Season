package frc.robot.subsystems.drive;

public interface MAXSwerveModuleIO {
    
   public void setDriveEncoderPosition(double position);
       
    public double getDriveEncoderPosition();
     
    public double getDriveEncoderSpeedMPS();
       
    public void setTurnEncoderPosition(double position);
    public double getTurnEncoderPosition();
    public void setDriveMotorSpeed(double speed);
    public void setDesiredDriveSpeedMPS(double speed);
}