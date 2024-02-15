package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.utils.MotorUtil;

public class RealIntake implements IntakeIO {

    public static CANSparkMax leftCenteringIntakeMotorController;
    public static CANSparkMax rightCenteringIntakeMotorController;
    public static CANSparkMax intakeMotorController;
    public static RelativeEncoder leftCenteringIntakeEncoder;
    public static RelativeEncoder rightCenteringIntakeEncoder;
    public static RelativeEncoder intakeEncoder;
    public static SparkPIDController leftCenteringIntakeController;
    public static SparkPIDController rightCenteringIntakeController;
    public static SparkPIDController intakeController;
    private double slewRate = 0.2;
    private static DigitalInput intakeSensorTop;
    private static DigitalInput intakeSensorBottom;

    public RealIntake()
    {
        leftCenteringIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.LEFT_CENTERING_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);
        
        rightCenteringIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.RIGHT_CENTERING_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, true, true, slewRate);

        intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_CENTERING_ID, MotorType.kBrushless,
        Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        leftCenteringIntakeEncoder = leftCenteringIntakeMotorController.getEncoder();
        rightCenteringIntakeEncoder = rightCenteringIntakeMotorController.getEncoder();
        intakeEncoder = intakeMotorController.getEncoder();

        leftCenteringIntakeController = leftCenteringIntakeMotorController.getPIDController();
        rightCenteringIntakeController = rightCenteringIntakeMotorController.getPIDController();
        intakeController = intakeMotorController.getPIDController();

        leftCenteringIntakeController.setFeedbackDevice(leftCenteringIntakeEncoder);
        rightCenteringIntakeController.setFeedbackDevice(rightCenteringIntakeEncoder);
        intakeController.setFeedbackDevice(intakeEncoder);

        intakeSensorTop = new DigitalInput(Constants.IntakeConstants.INTAKE_SENSOR_CHANNEL_TOP);
        intakeSensorBottom = new DigitalInput(Constants.IntakeConstants.INTAKE_SENSOR_CHANNEL_BOTTOM);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftCenteringIntakeMotorController.set(intakeSpeed);
        rightCenteringIntakeMotorController.set(intakeSpeed);
        intakeMotorController.set(intakeSpeed);
    }

    public boolean isIntooked() {
        return intakeSensorTop.get();
    }

    public void setSpeed(double speedPercent) {
        leftCenteringIntakeController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        rightCenteringIntakeController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        intakeController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        SmartDashboard.putNumber("shooter reference", speedPercent);
        SmartDashboard.putNumber("shooter speed (RPM)", getLeftEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM);
        SmartDashboard.putNumber("shooter speed (RPM)", getRightEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM);
        SmartDashboard.putNumber("shooter speed (RPM)", getIntakeEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM); 

    }

    public double getLeftCurrent()
    {
        return rightCenteringIntakeMotorController.getOutputCurrent();
    }

    public double getRightCurrent()
    {
        return leftCenteringIntakeMotorController.getOutputCurrent();
    }

    @Override
    public double getLeftEncoderSpeed() {
        return leftCenteringIntakeEncoder.getVelocity();
    }

    @Override
    public double getRightEncoderSpeed() {
        return rightCenteringIntakeEncoder.getVelocity();
    }

    public double getIntakeEncoderSpeed() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public double getLeftEncoderPosition() {
        return leftCenteringIntakeEncoder.getPosition();
    }

    @Override
    public double getRightEncoderPosition() {
        return rightCenteringIntakeEncoder.getPosition();
    }

    @Override
    public void setLeftCurrentLimit(int current) {
        leftCenteringIntakeMotorController.setSmartCurrentLimit(current);        
    }

    @Override
    public void setRightCurrentLimit(int current) {
        rightCenteringIntakeMotorController.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
       
    }

}
