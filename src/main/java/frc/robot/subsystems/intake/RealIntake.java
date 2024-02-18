package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

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
    private double slewRate = 0.2;

    public RealIntake()
    {
        leftCenteringIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.LEFT_CENTERING_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);
        
        rightCenteringIntakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.RIGHT_CENTERING_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_CENTERING_ID, MotorType.kBrushless,
        Constants.NEO550_CURRENT_LIMIT, true, true, slewRate);

        // initialize motor encoder
        leftCenteringIntakeEncoder = leftCenteringIntakeMotorController.getEncoder();
        rightCenteringIntakeEncoder = rightCenteringIntakeMotorController.getEncoder();
        intakeEncoder = intakeMotorController.getEncoder();
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftCenteringIntakeMotorController.set(intakeSpeed);
        rightCenteringIntakeMotorController.set(intakeSpeed);
        intakeMotorController.set(intakeSpeed);
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
