
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;



public class SwerveModule{
   
    private SwerveModuleState m_desiredState;

    double chassisAngularOffset;
    
    SwerveModuleIO io;


    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, SwerveModule io){
    

    io.setDriveEncoderPosition(0);
    m_desiredState.angle = new Rotation2d(getTurnEncoderPosition());
    this.io = io;


    }
    public void setDriveEncoderPosition(double position){
        io.setDriveEncoderPosition(position);
    }
    public double getDriveEncoderPosition(){
        io.getDriveEncoderPosition();
    }
    public double getDriveEncoderSpeedMPS(){
        io.getDriveEncoderSpeedMPS();
    }
    public void setDriveMotorSpeedMPS(double speed){
        io.setDriveMotorSpeed(speed);
    }
    public void setTurnEncoderPosition(double position){
        io.setTurnEncoderPosition(position);
    }
    public double getTurnEncoderPosition(){
        io.getTurnEncoderPosition();
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
    
    }
  public void periodic() {
    io.periodicUpdate();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(getDriveEncoderSpeedMPS(),
        new Rotation2d((getTurnEncoderPosition()) - chassisAngularOffset));
  }

   /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        getDriveEncoderPosition(),
        new Rotation2d(getTurnEncoderPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(getTurnEncoderPosition()));

    io.setDesiredDriveSpeedMPS(optimizedDesiredState.speedMetersPerSecond);
    io.setDesiredTurnAngle(optimizedDesiredState.angle.getRadians());
    
    m_desiredState = desiredState;
  }

}