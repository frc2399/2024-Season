package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static AbsoluteEncoder armAbsoluteEncoderRight;
    public static RelativeEncoder armEncoderRight;
    // private final SparkPIDController armPIDControllerRight;
    public static double speedFromArmHeight;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        // Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this)
        // translates to .2 seconds from 0 to 20% (what we actually use)

        armMotorControllerRight = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_RIGHT, MotorType.kBrushless,
                Constants.NEO_CURRENT_LIMIT,
                true, true, 0);

        armAbsoluteEncoderRight = armMotorControllerRight.getAbsoluteEncoder(Type.kDutyCycle);
        armEncoderRight = armMotorControllerRight.getEncoder();
        
        // armPIDControllerRight = armMotorControllerRight.getPIDController();
        // armPIDControllerRight.setFeedbackDevice(armAbsoluteEncoderRight);

        armAbsoluteEncoderRight.setPositionConversionFactor(ArmConstants.ABSOLUTE_RADIANS_PER_REVOLUTION);
        armAbsoluteEncoderRight.setVelocityConversionFactor(ArmConstants.ABSOLUTE_RADIANS_PER_REVOLUTION / 60);
        armAbsoluteEncoderRight.setInverted(true);
        armAbsoluteEncoderRight.setZeroOffset(-Math.PI / 2);
        armEncoderRight.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoderRight.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);
        

        armMotorControllerLeft = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID_LEFT, MotorType.kBrushless,
            Constants.NEO_CURRENT_LIMIT,
            false, true, 0);
        armMotorControllerLeft.follow(armMotorControllerRight, true);
        armEncoderRight.setPosition(armAbsoluteEncoderRight.getPosition());
        // armAbsoluteEncoderRight.setPosition(ArmConstants.INITIAL_OFFSET);

    }

    public double getAbsoluteEncoderPosition() {
        return armAbsoluteEncoderRight.getPosition();
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm position", getEncoderPosition());
        SmartDashboard.putNumber("arm absolute position", getAbsoluteEncoderPosition());
    }

    @Override
    public double getEncoderPosition() {
        // return getAbsoluteEncoderPosition();
        return armEncoderRight.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armAbsoluteEncoderRight.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerRight.set(speed);
    }

    @Override
    public void setPosition(double position) {
        //armPIDControllerRight.setReference(position, ControlType.kPosition);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerRight.getOutputCurrent();
    }

    @Override
    public void setEncoderPosition(double angle) {
       armEncoderRight.setPosition(angle);
    }

    @Override
    public double getSpeedFromArmHeight() {
        if (getEncoderPosition() < 0.4) {
            speedFromArmHeight = 0.7;
          } else if (getEncoderPosition() < 0.8 & getEncoderPosition() > 0.4) {
            speedFromArmHeight = 0.8;
          } else if (getEncoderPosition() < 1 & getEncoderPosition() > 0.8) {
            speedFromArmHeight = 1;
          } else if (getEncoderPosition() > 1) {
              speedFromArmHeight = 0.3;
          }
          return speedFromArmHeight;
    }
}
