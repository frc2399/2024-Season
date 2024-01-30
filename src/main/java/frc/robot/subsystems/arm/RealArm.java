package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static RelativeEncoder armEncoderLeft;
    public static RelativeEncoder armEncoderRight;
    public static DutyCycleEncoder armAbsoluteEncoder;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        // Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this)
        // translates to .2 seconds from 0 to 20% (what we actually use)
        armMotorControllerLeft = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_LEFT, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT,
                false, true, 0.75);
        armEncoderLeft = armMotorControllerLeft.getEncoder();

        armEncoderLeft.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoderLeft.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        // armEncoderLeft.setPosition(ArmConstants.INITIAL_OFFSET);

        armMotorControllerRight = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT,
                true, true, 0.75);
        armEncoderRight = armMotorControllerRight.getEncoder();

        armEncoderRight.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoderRight.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        // armEncoderRight.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    public double getAbsoluteEncoderPosition() {
        return -(armAbsoluteEncoder.getAbsolutePosition() - 0.88) * 2 * Math.PI / 3;
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/temp (C)", armMotorControllerLeft.getMotorTemperature());
    }

    @Override
    public double getEncoderPosition() {
        // return getAbsoluteEncoderPosition();
        return armEncoderLeft.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoderLeft.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerRight.set(speed);
        armMotorControllerLeft.set(speed);
    }

    @Override
    public void setPosition(double position) {
        armEncoderLeft.setPosition(position);
        armEncoderRight.setPosition(position);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerLeft.getOutputCurrent();
    }

}
