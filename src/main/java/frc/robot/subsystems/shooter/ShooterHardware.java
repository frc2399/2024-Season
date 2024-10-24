package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ShooterConstants;
import frc.utils.MotorUtil;

public class ShooterHardware implements ShooterIO {

    public static CANSparkMax shooterMotorControllerLow;
    public static CANSparkMax shooterMotorControllerHigh;
    public static RelativeEncoder shooterLowEncoder;
    public static RelativeEncoder shooterHighEncoder;
    public static SparkPIDController shooterHighController;
    public static SparkPIDController shooterLowController;

    public double feedforward = 0.011;
    public double pvalue = 0.01;
    private double slewRate = 0;

    public ShooterHardware() {
        shooterMotorControllerLow = new CANSparkMax(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless);
        shooterMotorControllerLow.restoreFactoryDefaults();
        shooterMotorControllerLow.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        shooterMotorControllerLow.setInverted(true);
        shooterMotorControllerLow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        shooterMotorControllerHigh = new CANSparkMax(ShooterConstants.SHOOT_HIGH_MOTOR_ID, MotorType.kBrushless);
        shooterMotorControllerHigh.restoreFactoryDefaults();
        shooterMotorControllerHigh.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        shooterMotorControllerHigh.setInverted(true);
        shooterMotorControllerHigh.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // initialize motor encoder
        shooterLowEncoder = shooterMotorControllerLow.getEncoder();
        shooterHighEncoder = shooterMotorControllerHigh.getEncoder();
        // TODO put in constants
        shooterHighEncoder.setVelocityConversionFactor(1 / 60.0); // convert to rps
        shooterLowEncoder.setVelocityConversionFactor(1 / 60.0); // convert to rps

        // initialize PID controllers, set feedback device
        shooterHighController = shooterMotorControllerHigh.getPIDController();
        shooterLowController = shooterMotorControllerLow.getPIDController();
        shooterHighController.setFeedbackDevice(shooterHighEncoder);
        shooterLowController.setFeedbackDevice(shooterLowEncoder);
        // shooter cannot go backwards
        shooterHighController.setOutputRange(0, 1);
        shooterLowController.setOutputRange(0, 1);
        // set gains for PID controllers
        shooterHighController.setFF(feedforward);
        shooterHighController.setP(pvalue);
        shooterLowController.setFF(feedforward);
        shooterLowController.setP(pvalue);
    }

    // Basic shooting command
    @Override
    public void setMotor(double shootSpeed) {
        shooterHighController.setReference(shootSpeed * ShooterConstants.SHOOT_MAX_SPEED_RPS, ControlType.kVelocity);
        shooterLowController.setReference(shootSpeed * ShooterConstants.SHOOT_MAX_SPEED_RPS, ControlType.kVelocity);
        SmartDashboard.putNumber("Shooter/shooter goal speed", shootSpeed * ShooterConstants.SHOOT_MAX_SPEED_RPS);
    }

    @Override
    public double getEncoderSpeed() {
        return shooterHighEncoder.getVelocity();
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("Shooter/shooter speed", getEncoderSpeed());
    }

}
