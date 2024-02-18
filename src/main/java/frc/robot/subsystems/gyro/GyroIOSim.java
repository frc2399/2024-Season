package frc.robot.subsystems.gyro;

public class GyroIOSim implements GyroIO {
    private double yaw = 0; 

    public double getYaw(){
        return yaw;
    }
    public void setYaw(double yaw) {
        this.yaw = yaw;
    }
    public void updateInputs(GyroIOInputs inputs){
        return ;

    }
}


