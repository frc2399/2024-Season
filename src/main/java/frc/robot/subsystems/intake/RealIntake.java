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

    public static CANSparkMax leftIntakeMotorController;
    public static CANSparkMax rightIntakeMotorController;
    public static RelativeEncoder leftIntakeEncoder;
    public static RelativeEncoder rightIntakeEncoder;
    public static SparkPIDController leftIntakeController;
    public static SparkPIDController rightIntakeController;
    private double slewRate = 0.2;
    private static DigitalInput intakeSensor;

    public RealIntake()
    {
        leftIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);
        
        rightIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        leftIntakeEncoder = leftIntakeMotorController.getEncoder();
        rightIntakeEncoder = rightIntakeMotorController.getEncoder();
        leftIntakeController = leftIntakeMotorController.getPIDController();
        rightIntakeController = rightIntakeMotorController.getPIDController();
        leftIntakeController.setFeedbackDevice(leftIntakeEncoder);
        rightIntakeController.setFeedbackDevice(rightIntakeEncoder);
        leftIntakeController.setFF(0.0001);
        rightIntakeController.setFF(0.0001);
        //intakeController.setP(1);
        intakeSensor = new DigitalInput(0);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftIntakeMotorController.set(intakeSpeed);
        rightIntakeMotorController.set(intakeSpeed);
    }

    public boolean isIntooked() {
        return intakeSensor.get();
    }

    public void setSpeed(double speedPercent) {
        leftIntakeController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        rightIntakeController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        SmartDashboard.putNumber("shooter reference", speedPercent);
        SmartDashboard.putNumber("shooter speed (RPM)", getLeftEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM);
        SmartDashboard.putNumber("shooter speed (RPM)", getRightEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM);

    }

    public double getLeftCurrent()
    {
        return rightIntakeMotorController.getOutputCurrent();
    }

    public double getRightCurrent()
    {
        return leftIntakeMotorController.getOutputCurrent();
    }

    @Override
    public double getLeftEncoderSpeed() {
        return leftIntakeEncoder.getVelocity();
    }

    @Override
    public double getRightEncoderSpeed() {
        return rightIntakeEncoder.getVelocity();
    }

    @Override
    public double getLeftEncoderPosition() {
        return leftIntakeEncoder.getPosition();
    }

    @Override
    public double getRightEncoderPosition() {
        return rightIntakeEncoder.getPosition();
    }

    @Override
    public void setLeftCurrentLimit(int current) {
        leftIntakeMotorController.setSmartCurrentLimit(current);        
    }

    @Override
    public void setRightCurrentLimit(int current) {
        rightIntakeMotorController.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
       
    }

}
