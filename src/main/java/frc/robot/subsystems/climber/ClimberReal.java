package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    // get info on slew rate and what motors are doing
    public static final GenericEntry slewRate = Shuffleboard.getTab("Params").addPersistent("Climber Slew Rate", 5.0)
            .getEntry();
    public static final GenericEntry leftClimberMotor = Shuffleboard.getTab("Driver")
            .addPersistent("Left Climber Motor", 0).getEntry();
    public static final GenericEntry rightClimberMotor = Shuffleboard.getTab("Driver")
            .addPersistent("Right Climber Motor", 0).getEntry();

    SlewRateLimiter filter;

    // private double climberSetpoint;

    public static final double CLIMBER_KP = 0;// 1.875;
    public static final double CLIMBER_KI = 0;// 0.006;
    public static final double CLIMBER_KD = 0;// 52.5;
    public static final double CLIMBER_KF = 0.000086; // 0.15;
    public static final double CLIMBER_KIZ = 0;
    public static final double CLIMBER_K_MAX_OUTPUT = 1;
    public static final double CLIMBER_K_MIN_OUTPUT = 0;
    public static final double CLIMBER_MAX_RPM = 5700;

    public ClimberReal() {

        // initialize motor controllers
        leftMotorController = MotorUtil.createSparkMAX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless, 50,
                true,
                true, 0.5);
        rightMotorController = MotorUtil.createSparkMAX(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless,
                50, false, true, 0.5);

        // initialize motor encoder
        leftEncoder = leftMotorController.getEncoder();
        rightEncoder = rightMotorController.getEncoder();

        // initialize debouncers
        leftDebouncer = new Debouncer(0.15);
        rightDebouncer = new Debouncer(0.15);

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
        leftMotorController.setInverted(false);
        rightMotorController.setInverted(false);

        // set encoder velocity to meters/second
        leftEncoder.setPositionConversionFactor(Constants.ClimberConstants.ENCODER_VELOCITY_MPS);
        rightEncoder.setPositionConversionFactor(Constants.ClimberConstants.ENCODER_VELOCITY_MPS);


        // reset encoders to zero
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);

        // get info on climber slew rate
        SmartDashboard.putNumber("Climber Slew Rate",
                SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));
        filter = new SlewRateLimiter(SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));
        System.out.println("Climber SlewRateLimiter "
                + SmartDashboard.getNumber("Climber Slew Rate", ClimberConstants.CLIMBER_SLEW));

    }

    @Override

    public void periodicUpdate() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Left Climber Height", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Climber Hieght", getRightEncoderPosition());
    }

    // left basic climbing with just speed
    public void setLeftSpeed(double speed) {
        if ((isLeftRetracted() && speed < 0)) {
            leftMotorController.set(0);
            leftClimberMotor.setDouble(0);
        } else {
            speed = filter.calculate(speed);
            leftMotorController.set(speed);
            leftClimberMotor.setDouble(speed);
        }

    }

    // left climbing with setpoint
    public void setLeftMotor(double setpoint) {
        if ((isLeftRetracted() && setpoint < 0) || (isLeftSideStalling() && !isRightSideStalling())) {
            leftMotorController.set(0);
            leftClimberMotor.setDouble(0);
        } else {

            leftPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
            // SmartDashboard.putNumber("Climber speed ", speed);
        }
    }

    // right basic climbing with just speed
    public void setRightSpeed(double speed) {
        if ((isRightRetracted() && speed < 0)) {
            rightMotorController.set(0);
            rightClimberMotor.setDouble(0);
        } else {
            speed = filter.calculate(speed);
            rightMotorController.set(speed);
            rightClimberMotor.setDouble(speed);
        }

    }

    // right climing with setpoint
    public void setRightMotor(double setpoint) {
        if ((isRightRetracted() && setpoint < 0) || (isRightSideStalling() && !isLeftSideStalling())) {
            rightMotorController.set(0);
            rightClimberMotor.setDouble(0);
        } else {

            rightPIDController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
            // SmartDashboard.putNumber("Climber speed ", speed);
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
        return leftDebouncer.calculate(Math.abs(leftEncoder.getVelocity()) < ClimberConstants.VELOCITY_THRESHHOLD);
    }

    // if the right side is stalling, tell climber to stop
    public boolean isRightSideStalling() {
        return rightDebouncer.calculate(Math.abs(rightEncoder.getVelocity()) < ClimberConstants.VELOCITY_THRESHHOLD);
    }
}
