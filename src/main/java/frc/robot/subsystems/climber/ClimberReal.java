package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.utils.MotorUtil;

public class ClimberReal implements ClimberIO {

    private CANSparkMax leftMotorController;
    private CANSparkMax rightMotorController;
    private RelativeEncoder leftEncoder, rightEncoder;
    private SparkPIDController leftPIDController, rightPIDController;
    private Debouncer leftDebouncer, rightDebouncer;

    public static final double CLIMBER_KP = 0;
    public static final double CLIMBER_KI = 0;
    public static final double CLIMBER_KD = 0;
    public static final double CLIMBER_KF = 0.000086;
    public static final double CLIMBER_KIZ = 0;
    public static final double CLIMBER_K_MAX_OUTPUT = 1;
    public static final double CLIMBER_K_MIN_OUTPUT = 0;
    public static final double CLIMBER_MAX_RPM = 5700;

    public ClimberReal() {

        //TODO no more PID
        // initialize motor controllers
        leftMotorController = MotorUtil.createSparkMAX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless, 50,
                true,
                true, 0.5);
        rightMotorController = MotorUtil.createSparkMAX(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless,
                50, true, true, 0.5);

        // initialize motor encoders
        leftEncoder = leftMotorController.getEncoder();
        rightEncoder = rightMotorController.getEncoder();

        // initialize debouncers
        leftDebouncer = new Debouncer(0.025);
        rightDebouncer = new Debouncer(0.025);

        // converts encoder rotations to distance (meters)
        leftEncoder.setPositionConversionFactor(Constants.ClimberConstants.ENCODER_METERS);
        rightEncoder.setPositionConversionFactor(Constants.ClimberConstants.ENCODER_METERS);

        // initialize motor pid controllers
        leftPIDController = leftMotorController.getPIDController();
        rightPIDController = rightMotorController.getPIDController();

        // assigns values to PID controllers
        leftPIDController.setP(CLIMBER_KP);
        leftPIDController.setI(CLIMBER_KI);
        leftPIDController.setD(CLIMBER_KD);
        leftPIDController.setIZone(CLIMBER_KIZ);
        leftPIDController.setFF(CLIMBER_KF);
        leftPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

        rightPIDController.setP(CLIMBER_KP);
        rightPIDController.setI(CLIMBER_KI);
        rightPIDController.setD(CLIMBER_KD);
        rightPIDController.setIZone(CLIMBER_KIZ);
        rightPIDController.setFF(CLIMBER_KF);
        rightPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

        // invert the motor controllers so climber climbs right
        leftMotorController.setInverted(true);
        rightMotorController.setInverted(true);

        // set encoder velocity to meters/second
        leftEncoder.setVelocityConversionFactor(Constants.ClimberConstants.ENCODER_VELOCITY_MPS);
        rightEncoder.setVelocityConversionFactor(Constants.ClimberConstants.ENCODER_VELOCITY_MPS);


        // reset encoders to zero
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    @Override

    public void periodicUpdate() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("climber/Left Climber Height", getLeftEncoderPosition());
        SmartDashboard.putNumber("climber/Right Climber Hieght", getRightEncoderPosition());
        SmartDashboard.putNumber("climber/left climber current", leftMotorController.getOutputCurrent());
    }

    // left basic climbing with just speed
    public void setLeftSpeed(double speed) {
        if (((isLeftRetracted() && speed < 0)) || ((isLeftExtended() && speed > 0)) || (isLeftSideStalling() && !isRightSideStalling())) {
            leftMotorController.set(0);
        } else {
            leftMotorController.set(speed);
        }

    }

    // left climbing with setpoint
    public void setLeftMotor(double setpoint) {
        if ((isLeftRetracted() && setpoint < 0) || (isLeftSideStalling() && !isRightSideStalling())) {
            leftMotorController.set(0);
        } else {
            leftPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
        }
    }

    // right basic climbing with just speed
    public void setRightSpeed(double speed) {
        if ((isRightRetracted() && speed < 0) || ((isRightExtended() && speed > 0)) || (isRightSideStalling() && !isLeftSideStalling())) {
            rightMotorController.set(0);
        } else {
            rightMotorController.set(speed);
        }

    }

    // right climing with setpoint
    public void setRightMotor(double setpoint) {
        if ((isRightRetracted() && setpoint < 0) || (isRightSideStalling() && !isLeftSideStalling())) {
            rightMotorController.set(0);
        } else {

            rightPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
        }
    }

    public boolean isLeftExtended() {
        return (leftEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isRightExtended() {
        return (rightEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isLeftRetracted() {
        return (leftEncoder.getPosition() < ClimberConstants.MIN_HEIGHT);
    }

    public boolean isRightRetracted() {
        return (rightEncoder.getPosition() < ClimberConstants.MIN_HEIGHT);
    }

    public double getLeftEncoderPosition() {
        // gets position in inches
        double distance = (double) leftEncoder.getPosition();
        return distance;
    }

    public double getRightEncoderPosition() {
        // gets position in inches
        double distance = (double) rightEncoder.getPosition();
        return distance;
    }

    // if the left side is stalling, tell climber to stop
    public boolean isLeftSideStalling() {
        return leftDebouncer.calculate(Math.abs(leftMotorController.getOutputCurrent()) > ClimberConstants.CURRENT_THRESHOLD);
    }

    // if the right side is stalling, tell climber to stop
    public boolean isRightSideStalling() {
        return rightDebouncer.calculate(Math.abs(rightMotorController.getOutputCurrent()) > ClimberConstants.CURRENT_THRESHOLD);
    }
}
