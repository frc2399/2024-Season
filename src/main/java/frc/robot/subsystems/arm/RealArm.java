package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    public static CANSparkMax armMotorController;
    public static RelativeEncoder armEncoder;
    public static DutyCycleEncoder armAbsoluteEncoder;
    public static SparkPIDController armPidController;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        //Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this) translates to .2 seconds from 0 to 20% (what we actually use)
        armMotorController = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0.75);
        armEncoder = armMotorController.getEncoder();
        armPidController = armMotorController.getPIDController();

        //TODO: set gains for armPidController.
        armPidController.setP(1.6); //0.4
        armPidController.setD(0);
        armPidController.setI(0);
        armPidController.setFF(0);
        
        armEncoder.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoder.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        armEncoder.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    public double getAbsoluteEncoderPosition() {
        return -(armAbsoluteEncoder.getAbsolutePosition() - 0.88) * 2 * Math.PI / 3;
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/temp (C)", armMotorController.getMotorTemperature());
    }

    @Override
    public double getEncoderPosition() {
       //return getAbsoluteEncoderPosition();
        return armEncoder.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoder.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorController.set(speed);
    }

    @Override
    public void setPosition(double position) {
        armEncoder.setPosition(position);
    }

    @Override
    public double getArmCurrent() {
        return armMotorController.getOutputCurrent();
    }

    @Override
    public void setSetpoint(State setpoint, double feedforward) {
        // Set the setpoint
        armPidController.setReference(
            setpoint.position, 
            ControlType.kPosition, 
            0, 
            feedforward, 
            ArbFFUnits.kPercentOut);
        
    }

    @Override
    public double getMotorDutyCycle()
    {
        return armMotorController.getAppliedOutput();
    };

}
