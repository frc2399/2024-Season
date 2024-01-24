package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.MotorUtil;

public class RealShooter implements ShooterIO {

    public static CANSparkMax shooterMotorControllerLow;
    public static CANSparkMax shooterMotorControllerHigh;
    public static RelativeEncoder shooterLowEncoder;
    public static RelativeEncoder shooterHighEncoder;
    public static SparkPIDController shooterHighController;
    public static SparkPIDController shooterLowController;
    private double slewRate = 0.2;

    public RealShooter()
    {
        shooterMotorControllerLow = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, slewRate);
        
        shooterMotorControllerHigh = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_HIGH_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, slewRate);

        // initialize motor encoder
        shooterLowEncoder = shooterMotorControllerLow.getEncoder();
        shooterHighEncoder = shooterMotorControllerHigh.getEncoder();

        //initialize PID controllers, set gains
        shooterHighController = shooterMotorControllerHigh.getPIDController();
        shooterLowController = shooterMotorControllerLow.getPIDController();
        shooterHighController.setFeedbackDevice(shooterHighEncoder);
        shooterLowController.setFeedbackDevice(shooterLowEncoder);
        shooterLowController.setFF(.0001);
        shooterHighController.setFF(.0001);
        // shooterHighController.setP(.00045);
        // shooterLowController.setP(.00045);



    }

    //Basic shooting command
    @Override
    public void setMotor(double shootSpeed) {
        shooterMotorControllerLow.set(shootSpeed);
        shooterMotorControllerHigh.set(shootSpeed);
    }

    //Shooting command, but using PID
    @Override
    public void setSpeed(double speedPercent) {
        shooterHighController.setReference(speedPercent * Constants.NEO_MAX_SPEED_RPM, ControlType.kVelocity);
        shooterLowController.setReference(speedPercent * Constants.NEO_MAX_SPEED_RPM, ControlType.kVelocity);    
        SmartDashboard.putNumber("shooter reference", speedPercent);
        SmartDashboard.putNumber("shooter speed (RPM)", getEncoderSpeed() / Constants.NEO_MAX_SPEED_RPM);

    }

    public double getCurrent()
    {
        return shooterMotorControllerHigh.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return shooterHighEncoder.getVelocity();
    }

    @Override
    public void setCurrentLimit(int current) {
        shooterMotorControllerHigh.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
        // SmartDashboard.putNumber("intake/current (A)", getCurrent());
        // SmartDashboard.putNumber("intake/temp (C)", shooterMotorControllerHigh.getMotorTemperature());        
    }

}
