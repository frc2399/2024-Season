package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModuleIO {
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveBusVoltage = 0.0;
    
        public double turnPositionRad = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnBusVoltage = 0.0; 
    }

    public void updateInputs(SwerveModuleIOInputs inputs);
    
    public void setDriveEncoderPosition(double position);
    public double getDriveEncoderPosition();

    public void setDesiredDriveSpeedMPS(double speed);
    public double getDriveEncoderSpeedMPS();
       
    public double getTurnEncoderPosition();
    public void setDesiredTurnAngle(double angle);
    public double getDriveBusVoltage();
    public double getDriveOutput();
    public double getTurnBusVoltage(); 
    public double getTurnOutput();

    public double getChassisAngularOffset();

    public String getName();
}