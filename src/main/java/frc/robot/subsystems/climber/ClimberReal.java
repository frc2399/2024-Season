package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

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

        // TODO no more PID
        // initialize motor controllers
        leftMotorController = MotorUtil.createSparkMAX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless,
                50, true, true, 0.0);
        rightMotorController = MotorUtil.createSparkMAX(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless,
                50, true, true, 0.0);

        // initialize motor encoders
        leftEncoder = leftMotorController.getEncoder();
        rightEncoder = rightMotorController.getEncoder();

        // initialize debouncers
        leftDebouncer = new Debouncer(0.025);
        rightDebouncer = new Debouncer(0.025);

        // converts encoder rotations to distance (meters)
        leftEncoder.setPositionConversionFactor(Constants.ClimberConstants.ENCODER_METERS);
        rightEncoder.setPositionConversionFactor(Constants.ClimberConstants.ENCODER_METERS);

        leftMotorController.setSoftLimit(SoftLimitDirection.kReverse, (float) (ClimberConstants.MIN_HEIGHT + .01));
        leftMotorController.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightMotorController.setSoftLimit(SoftLimitDirection.kReverse, (float) (ClimberConstants.MIN_HEIGHT + .01));
        rightMotorController.enableSoftLimit(SoftLimitDirection.kReverse, true);

        leftMotorController.setSoftLimit(SoftLimitDirection.kForward, (float) (ClimberConstants.MAX_HEIGHT - .01));
        leftMotorController.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightMotorController.setSoftLimit(SoftLimitDirection.kForward, (float) (ClimberConstants.MAX_HEIGHT - .01));
        rightMotorController.enableSoftLimit(SoftLimitDirection.kForward, true);

        // initialize motor pid controllers
        leftPIDController = leftMotorController.getPIDController();
        rightPIDController = rightMotorController.getPIDController();

        // assigns values to PID controllers
        // leftPIDController.setP(CLIMBER_KP);
        // leftPIDController.setI(CLIMBER_KI);
        // leftPIDController.setD(CLIMBER_KD);
        // leftPIDController.setIZone(CLIMBER_KIZ);
        // leftPIDController.setFF(CLIMBER_KF);
        // leftPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT, CLIMBER_K_MAX_OUTPUT);

        // rightPIDController.setP(CLIMBER_KP);
        // rightPIDController.setI(CLIMBER_KI);
        // rightPIDController.setD(CLIMBER_KD);
        // rightPIDController.setIZone(CLIMBER_KIZ);
        // rightPIDController.setFF(CLIMBER_KF);
        // rightPIDController.setOutputRange(CLIMBER_K_MIN_OUTPUT,
        // CLIMBER_K_MAX_OUTPUT);

        // invert the motor controllers so climber climbs right
        leftMotorController.setInverted(false);
        rightMotorController.setInverted(true);

        // set encoder velocity to meters/second
        leftEncoder.setVelocityConversionFactor(Constants.ClimberConstants.ENCODER_VELOCITY_MPS);
        rightEncoder.setVelocityConversionFactor(Constants.ClimberConstants.ENCODER_VELOCITY_MPS);

        // reset encoders to zero
        // leftEncoder.setPosition(0.0);
        // rightEncoder.setPosition(0.0);

        if (leftEncoder.setPosition(0.0) != REVLibError.kOk) {
            System.out.println("****************left encoder set position is not successful");
        }

        if (rightEncoder.setPosition(0.0) != REVLibError.kOk) {
            System.out.println("****************right encoder set position is not successful");
        }
    }

    @Override

    public void periodicUpdate() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("climber/Left Climber Height", getLeftEncoderPosition());
        SmartDashboard.putNumber("climber/Right Climber Hieght", getRightEncoderPosition());
        SmartDashboard.putNumber("climber/left climber current", leftMotorController.getOutputCurrent());
        SmartDashboard.putBoolean("climber/is left retracted", isLeftRetracted());
        SmartDashboard.putNumber("climber/right climber current", rightMotorController.getOutputCurrent());
        SmartDashboard.putBoolean("climber/is right retracted", isRightRetracted());
        SmartDashboard.putNumber("climber/leftspeed", leftMotorController.getAppliedOutput());
        SmartDashboard.putBoolean("climber/is left extended", isLeftExtended());
        SmartDashboard.putBoolean("climber/is right extended", isRightExtended());
        SmartDashboard.putBoolean("climber/is left side stalling", isLeftSideStalling());
        SmartDashboard.putBoolean("climber/is right side stalling", isRightSideStalling());

    }

    // left basic climbing with just speed
    public void setLeftSpeed(double speed) {

        SmartDashboard.putBoolean("climber/at bottom left", (isLeftRetracted() && speed < 0));
        SmartDashboard.putBoolean("climber/at top left", (isLeftExtended() && speed > 0));
        SmartDashboard.putBoolean("climber/left stalling", (isLeftSideStalling() && !isRightSideStalling()));
        if ((isLeftRetracted() && speed < 0) || ((isLeftExtended() && speed > 0))
                || (isLeftSideStalling() && !isRightSideStalling() && speed < 0)) {
            leftMotorController.set(0);
        } else if (leftEncoder.getPosition() < 0.10 && speed < 0) {
            leftMotorController.set(-0.15);
        } else {
            leftMotorController.set(speed);
        }

    }

    // right basic climbing with just speed
    public void setRightSpeed(double speed) {
        if ((isRightRetracted() && speed < 0) || ((isRightExtended() && speed > 0))
                || (isRightSideStalling() && !isLeftSideStalling() && speed < 0)) {
            rightMotorController.set(0);
        } else if (rightEncoder.getPosition() < 0.10 && speed < 0) {
            rightMotorController.set(-0.15);
        } else {
            rightMotorController.set(speed);
        }

    }

    // right climing with setpoint

    public boolean isLeftExtended() {
        return (leftEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isRightExtended() {
        return (rightEncoder.getPosition() > ClimberConstants.MAX_HEIGHT);
    }

    public boolean isLeftRetracted() {
        return (leftEncoder.getPosition() < ClimberConstants.MIN_HEIGHT + .01);
    }

    public boolean isRightRetracted() {
        return (rightEncoder.getPosition() < ClimberConstants.MIN_HEIGHT + .01);
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
        return leftDebouncer
                .calculate(Math.abs(leftMotorController.getOutputCurrent()) > ClimberConstants.CURRENT_THRESHOLD);
    }

    // if the right side is stalling, tell climber to stop
    public boolean isRightSideStalling() {
        return rightDebouncer
                .calculate(Math.abs(rightMotorController.getOutputCurrent()) > ClimberConstants.CURRENT_THRESHOLD);
    }
}
