package frc.robot.subsystems.shooter2;

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

    public static CANSparkMax shooterMotorControllerLeft;
    public static CANSparkMax shooterMotorControllerRight;
    public static RelativeEncoder shooterLeftEncoder;
    public static RelativeEncoder shooterRightEncoder;
    public static SparkPIDController shooterRightController;
    public static SparkPIDController shooterLeftController;
    private double slewRate = 0.2;

    public RealShooter()
    {
        shooterMotorControllerLeft = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, slewRate);
        
        shooterMotorControllerRight = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_HIGH_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, slewRate);

        // initialize motor encoder
        shooterLeftEncoder = shooterMotorControllerLeft.getEncoder();
        shooterRightEncoder = shooterMotorControllerRight.getEncoder();

        //initialize PID controllers, set gains
        shooterRightController = shooterMotorControllerRight.getPIDController();
        shooterLeftController = shooterMotorControllerLeft.getPIDController();
        shooterRightController.setFeedbackDevice(shooterRightEncoder);
        shooterLeftController.setFeedbackDevice(shooterLeftEncoder);
        shooterLeftController.setFF(.0001);
        shooterRightController.setFF(.0001);
        // shooterHighController.setP(.00045);
        // shooterLowController.setP(.00045);



    }

    //Basic shooting command
    @Override
    public void setMotor(double shootSpeed) {
        shooterMotorControllerLeft.set(shootSpeed);
        shooterMotorControllerRight.set(shootSpeed);
    }

    //Shooting command, but using PID
    @Override
    public void setSpeed(double speedPercent) {
        shooterRightController.setReference(speedPercent * Constants.NEO_MAX_SPEED_RPM, ControlType.kVelocity);
        shooterLeftController.setReference((speedPercent - (SmartDashboard.getNumber("Shoot2 speed", 0)))* Constants.NEO_MAX_SPEED_RPM, ControlType.kVelocity);    
        SmartDashboard.putNumber("shooter reference", speedPercent);
        SmartDashboard.putNumber("shooter speed (RPM)", getEncoderSpeed() / Constants.NEO_MAX_SPEED_RPM);
    }

    public double getCurrent()
    {
        return shooterMotorControllerRight.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return shooterRightEncoder.getVelocity();
    }

    @Override
    public void setCurrentLimit(int current) {
        shooterMotorControllerRight.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
        // SmartDashboard.putNumber("intake/current (A)", getCurrent());
        // SmartDashboard.putNumber("intake/temp (C)", shooterMotorControllerHigh.getMotorTemperature());        
    }

}
