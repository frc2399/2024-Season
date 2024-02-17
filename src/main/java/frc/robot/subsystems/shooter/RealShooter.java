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

    public double feedforward = 0;
    public double pvalue = 0.001;
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



    }

    //Basic shooting command
    @Override
    public void setMotor(double shootSpeed) {
        shooterHighController.setReference(shootSpeed, ControlType.kDutyCycle);
        shooterLowController.setReference(shootSpeed, ControlType.kDutyCycle);   

    }

    //Shooting command, but using PID
    @Override
    public void setSpeed(double speedPercent) {
        shooterHighController.setReference(speedPercent, ControlType.kVelocity);
        shooterLowController.setReference(speedPercent, ControlType.kVelocity);    
        SmartDashboard.putNumber("shooter reference", speedPercent);
        SmartDashboard.putNumber("shooter speed (RPM)", getEncoderSpeed() / Constants.ShooterConstants.NEO_MAX_SPEED_MPS);

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

        feedforward = SmartDashboard.getNumber("shooter/ff", feedforward);    
        pvalue = SmartDashboard.getNumber("shooter/pvalue", pvalue); 
        SmartDashboard.putNumber("shooter/pvalue", pvalue);
        shooterLowController.setFF(feedforward);
        shooterHighController.setFF(feedforward);
        shooterHighController.setP(pvalue);
        shooterLowController.setP(pvalue);
 
    }

}
