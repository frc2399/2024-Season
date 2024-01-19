package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    private double slewRate = 0.2;

    public RealShooter() {
        shooterMotorControllerLow = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_LOW_MOTOR_ID, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT, false, true, slewRate);

        shooterMotorControllerHigh = MotorUtil.createSparkMAX(ShooterConstants.SHOOT_HIGH_MOTOR_ID,
                MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        shooterLowEncoder = shooterMotorControllerLow.getEncoder();
        shooterHighEncoder = shooterMotorControllerHigh.getEncoder();
    }

    @Override
    public void setMotor(double shootSpeed) {
        shooterMotorControllerLow.set(shootSpeed);
        shooterMotorControllerHigh.set(-shootSpeed);
    }

    public double getCurrent() {
        return shooterMotorControllerLow.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return shooterLowEncoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return shooterLowEncoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        shooterLowEncoder.setPosition(position);
    }

    @Override
    public void setCurrentLimit(int current) {
        shooterMotorControllerLow.setSmartCurrentLimit(current);
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("intake/current (A)", getCurrent());
        SmartDashboard.putNumber("intake/temp (C)", shooterMotorControllerLow.getMotorTemperature());
    }

}
