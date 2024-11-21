package frc.robot.subsystems.arm;

import org.photonvision.PhotonUtils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static AbsoluteEncoder armAbsoluteEncoderRight;
    public static RelativeEncoder armEncoderRight;
    public static double speedFromArmHeight;
    final static double EIGHTYSLOPE = VisionConstants.EIGHTYMODELSLOPE;
    final static double EIGHTYINTERCEPT = VisionConstants.EIGHTYMODELINTERCEPT;
    final static double HUNDREDSLOPE = VisionConstants.HUNDREDMODELSLOPE;
    final static double HUNDREDINTERCEPT = VisionConstants.HUNDREDMODELINTERCEPT;
    final static double BOUNDARY = VisionConstants.EIGHTYMODELRANGE;
    final static double STAYDOWNBOUNDARY = VisionConstants.STAYDOWNBOUNDARY;
    final static double DISTANCE_TO_CENTER_FROM_FRAME_INCHES = 15.75;

    private static final int ARM_MOTOR_ID_LEFT = 9;
    private static final int ARM_MOTOR_ID_RIGHT = 10;

    private static final double ABSOLUTE_RADIANS_PER_REVOLUTION = 2 * Math.PI / 4;
    // calculations for conversion factor: 1 4-1 gearbox, 2 3-1 gearboxes,
    // then a 4-1 reduction from the sprocket/chain;
    // 4^2 * 3^2 = 144
    private static final double RADIANS_PER_REVOLUTION = 2 * Math.PI / 144;
    private static final double ARM_ABSOLUTE_MEASURED = 1.03;
    private static final double ARM_ABSOLUTE_CAD = 0.274;

    public RealArm() {
        armMotorControllerRight = new CANSparkMax(ARM_MOTOR_ID_RIGHT, MotorType.kBrushless);
        armMotorControllerLeft = new CANSparkMax(ARM_MOTOR_ID_LEFT, MotorType.kBrushless);

        armMotorControllerRight.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        armMotorControllerLeft.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        armAbsoluteEncoderRight = armMotorControllerRight.getAbsoluteEncoder(Type.kDutyCycle);
        armAbsoluteEncoderRight.setPositionConversionFactor(ABSOLUTE_RADIANS_PER_REVOLUTION);
        armAbsoluteEncoderRight.setVelocityConversionFactor(ABSOLUTE_RADIANS_PER_REVOLUTION / 60);
        armAbsoluteEncoderRight.setInverted(true);
        armAbsoluteEncoderRight.setZeroOffset(ARM_ABSOLUTE_MEASURED - ARM_ABSOLUTE_CAD);

        armEncoderRight = armMotorControllerRight.getEncoder();
        armEncoderRight.setPositionConversionFactor(RADIANS_PER_REVOLUTION);
        armEncoderRight.setVelocityConversionFactor(RADIANS_PER_REVOLUTION / 60);
        armEncoderRight.setPosition(armAbsoluteEncoderRight.getPosition());

        // set the left motor to follow the right one, but inverted since left isn't
        armMotorControllerLeft.follow(armMotorControllerRight, true);
        System.out.println("real arm being constructed");
    }

    public double getEncoderPosition() {
        return armEncoderRight.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return armEncoderRight.getVelocity();
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/actual position (deg)", Math.toDegrees(getEncoderPosition()));
        SmartDashboard.putNumber("arm/absolute position (deg)", Math.toDegrees(getAbsoluteEncoderPosition()));
        SmartDashboard.putNumber("arm/actual velocity (deg per s)", Math.toDegrees(getEncoderVelocity()));
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerRight.getOutputCurrent();
    }

    @Override
    public double getAbsoluteEncoderPosition() {
        return armAbsoluteEncoderRight.getPosition();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerRight.set(speed);
    }

    @Override
    public void setEncoderPosition(double angle) {
        armEncoderRight.setPosition(angle);
    }

    public double getDesiredArmAngle(Pose2d robotPose, Pose2d speakerPose) {
        double distToSpeaker;
        double desiredArmAngleRadians;
        distToSpeaker = PhotonUtils.getDistanceToPose(robotPose, speakerPose);
        if (distToSpeaker <= STAYDOWNBOUNDARY) {
            desiredArmAngleRadians = 0.31;
        } else if (distToSpeaker <= BOUNDARY) {
            desiredArmAngleRadians = EIGHTYSLOPE * (distToSpeaker) + EIGHTYINTERCEPT;
        } else {
            desiredArmAngleRadians = HUNDREDSLOPE * (distToSpeaker) + HUNDREDINTERCEPT;
        }
        return desiredArmAngleRadians;
    }
}
