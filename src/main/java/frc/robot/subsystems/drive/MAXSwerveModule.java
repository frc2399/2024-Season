
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkLowLevel.MotorType;



public class MAXSwerveModule{
   
    private SwerveModuleState m_desiredState;

    double chassisAngularOffset;

    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, MAXSwerveModule io){
    

    io.setDriveEncoderPosition(0);
    m_desiredState.angle = new Rotation2d(getTurnEncoderPosition());


}
public void setDriveEncoderPosition{
    io.setDriveEncoderPosition;
}
public void getDriveEncoderPosition{
    io.getDriveEncoderPosition;
}
public void getDriveEncoderSpeedMPS{
    io.getDriveEncoderSpeedMPS;
}
public void setTurnEncoderPosition{
    io.setTurnEncoderPosition;
}
public void getTurnEncoderPosition{
    io.getTurnEncoderPosition;
}
public void setDriveMotorSpeed{
    io.setDriveMotorSpeed;
}
public void setDesiredState(SwerveModuleState desiredState) {
   
  }
  public void periodic() {
    io.periodicUpdate();
  }
}