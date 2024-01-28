package frc.robot.subsystems.drive;

public interface SwerveModuleIO {
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
    
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
    }

    public void updateInputs(SwerveModuleIOInputs inputs);
    
   public void setDriveEncoderPosition(double position);
       
    public double getDriveEncoderPosition();
     
    public double getDriveEncoderSpeedMPS();
       
    public void setTurnEncoderPosition(double position);
    public double getTurnEncoderPosition();
    public void setDriveMotorSpeed(double speed);
    public void setDesiredDriveSpeedMPS(double speed);
    public void setDesiredTurnAngle(double angle);
}