package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.utils.MotorUtil;

public class RealIntake implements IntakeIO {

    public static CANSparkMax intakeMotorController;
    public static RelativeEncoder intakeEncoder;
    public static SparkPIDController intakeController;
    private double slewRate = 0.2;

    public RealIntake()
    {
        intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        intakeEncoder = intakeMotorController.getEncoder();
        intakeController = intakeMotorController.getPIDController();
        intakeController.setFeedbackDevice(intakeEncoder);
        intakeController.setFF(0.0001);
        //intakeController.setP(1);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        intakeMotorController.set(intakeSpeed);
    }

    public void setSpeed(double speedPercent) {
        intakeController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        SmartDashboard.putNumber("shooter reference", speedPercent);
        SmartDashboard.putNumber("shooter speed (RPM)", getEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM);

    }

    public double getCurrent()
    {
        return intakeMotorController.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    @Override
    public void setCurrentLimit(int current) {
        intakeMotorController.setSmartCurrentLimit(current);        
    }

    @Override
    public void periodicUpdate() {
       
    }

}
