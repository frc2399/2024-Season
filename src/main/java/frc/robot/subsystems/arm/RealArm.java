package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static AbsoluteEncoder armAbsoluteEncoderRight;
    public static RelativeEncoder armEncoderRight;
    public static double speedFromArmHeight;

    public static final int ARM_MOTOR_ID_LEFT = 9;
    public static final int ARM_MOTOR_ID_RIGHT = 10;

    public RealArm() {
        // make the motor controllers
        armMotorControllerRight = MotorUtil.createSparkMAX(ARM_MOTOR_ID_RIGHT, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT,
                true, true, 0);
        armMotorControllerLeft = MotorUtil.createSparkMAX(ARM_MOTOR_ID_LEFT, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT,
                false, true, 0);

        // make the encoders
        // .044
        armAbsoluteEncoderRight = armMotorControllerRight.getAbsoluteEncoder(Type.kDutyCycle);
        armEncoderRight = armMotorControllerRight.getEncoder();

        // setting position/velocity conversion factors, offsets
        armAbsoluteEncoderRight.setPositionConversionFactor(ArmConstants.ABSOLUTE_RADIANS_PER_REVOLUTION);
        armAbsoluteEncoderRight.setVelocityConversionFactor(ArmConstants.ABSOLUTE_RADIANS_PER_REVOLUTION / 60);
        armAbsoluteEncoderRight.setInverted(true);
        // armAbsoluteEncoderRight.setZeroOffset(0);
        armAbsoluteEncoderRight.setZeroOffset(ArmConstants.ARM_ABSOLUTE_MEASURED - ArmConstants.ARM_ABSOLUTE_CAD);
        armEncoderRight.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoderRight.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);
        armEncoderRight.setPosition(armAbsoluteEncoderRight.getPosition());

        // set the left motor to follow the right one, but inverted since left isn't
        // reversed and right is
        armMotorControllerLeft.follow(armMotorControllerRight, true);
    }

    public double getAbsoluteEncoderPosition() {
        return armAbsoluteEncoderRight.getPosition();
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/actual position (deg)", Math.toDegrees(getEncoderPosition()));
        SmartDashboard.putNumber("arm/absolute position (deg)", Math.toDegrees(getAbsoluteEncoderPosition()));
        SmartDashboard.putNumber("arm/actual velocity (deg per s)", Math.toDegrees(getEncoderSpeed()));
    }

    @Override
    public double getEncoderPosition() {
        return armEncoderRight.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoderRight.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerRight.set(speed);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerRight.getOutputCurrent();
    }

    @Override
    public void setEncoderPosition(double angle) {
        armEncoderRight.setPosition(angle);
    }
}
