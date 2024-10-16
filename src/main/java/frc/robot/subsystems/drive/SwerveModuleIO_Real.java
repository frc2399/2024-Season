package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.utils.MotorUtil;

public class SwerveModuleIO_Real implements SwerveModuleIO {
   private final CANSparkMax drivingSparkMax;
   private final CANSparkMax turningSparkMax;

   private final RelativeEncoder drivingEncoder;
   private final AbsoluteEncoder turningEncoder;

   private final SparkPIDController drivingPIDController;
   private final SparkPIDController turningPIDController;

   private double chassisAngularOffset;

   private String name;

   public SwerveModuleIO_Real(int drivingCANId, int turningCANId, double chassisAngularOffset,
         String name) {

      int errors = 0;

      this.name = name;

      drivingSparkMax = MotorUtil.createSparkMAX(drivingCANId, MotorType.kBrushless,
            Constants.NEO_CURRENT_LIMIT, SwerveModuleConstants.drivingEncoderInverted, true, 0);
      turningSparkMax = MotorUtil.createSparkMAX(turningCANId, MotorType.kBrushless,
            Constants.NEO550_CURRENT_LIMIT, true, 0);

      errors += check(drivingSparkMax.enableVoltageCompensation(12));
      errors += check(turningSparkMax.enableVoltageCompensation(12));

      // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
      drivingEncoder = drivingSparkMax.getEncoder();
      turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      drivingPIDController = drivingSparkMax.getPIDController();
      turningPIDController = turningSparkMax.getPIDController();

      errors += check(drivingPIDController.setFeedbackDevice(drivingEncoder));
      errors += check(turningPIDController.setFeedbackDevice(turningEncoder));

      // Apply position and velocity conversion factors for the driving encoder. The
      // native units for position and velocity are rotations and RPM, respectively,
      // but we want meters and meters per second to use with WPILib's swerve APIs.

      errors += check(
            drivingEncoder
                  .setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor));
      errors += check(
            drivingEncoder.setVelocityConversionFactor(
                  (SwerveModuleConstants.kDrivingEncoderVelocityFactor)));

      // Apply position and velocity conversion factors for the turning encoder. We
      // want these in radians and radians per second to use with WPILib's swerve
      // APIs.
      errors += check(
            turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor));
      errors += check(
            turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor));

      // Invert the turning encoder, since the output shaft rotates in the opposite
      // direction of
      // the steering motor in the MAXSwerve Module.

      errors += check(turningEncoder.setInverted(SwerveModuleConstants.turningEncoderInverted));
      drivingSparkMax.setInverted(SwerveModuleConstants.drivingEncoderInverted);

      // Enable PID wrap around for the turning motor. This will allow the PID
      // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
      // to 10 degrees will go through 0 rather than the other direction which is a
      // longer route.

      errors += check(turningPIDController.setPositionPIDWrappingEnabled(true));
      errors += check(turningPIDController
            .setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput));
      errors += check(turningPIDController
            .setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput));

      // Set the PID gains for the driving motor
      errors += check(drivingPIDController.setP(SwerveModuleConstants.kDrivingP));
      errors += check(drivingPIDController.setI(SwerveModuleConstants.kDrivingI));
      errors += check(drivingPIDController.setD(SwerveModuleConstants.kDrivingD));
      errors += check(drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF));
      errors += check(drivingPIDController.setOutputRange(SwerveModuleConstants.kDrivingMinOutput,
            SwerveModuleConstants.kDrivingMaxOutput));

      // Set the PID gains for the turning motor
      errors += check(turningPIDController.setP(SwerveModuleConstants.kTurningP));
      errors += check(turningPIDController.setI(SwerveModuleConstants.kTurningI));
      errors += check(turningPIDController.setD(SwerveModuleConstants.kTurningD));
      errors += check(turningPIDController.setFF(SwerveModuleConstants.kTurningFF));
      errors += check(turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput,
            SwerveModuleConstants.kTurningMaxOutput));

      this.chassisAngularOffset = chassisAngularOffset;

      errors += check(drivingEncoder.setPosition(0));

      if (errors > 0) {
         System.out.println("Swerve Module Errors! Name: " + name + ", Amount: " + errors);
      }
   }

   public void updateInputs(SwerveModuleIOInputs inputs) {
      inputs.driveBusVoltage = getDriveBusVoltage();
      inputs.driveAppliedVolts = getDriveBusVoltage() * getDriveOutput();
      inputs.drivePositionMeters = getDriveEncoderPosition();
      inputs.driveVelocityMPS = getDriveEncoderSpeedMPS();
      inputs.turnBusVoltage = getTurnBusVoltage();
      inputs.turnAppliedVolts = getTurnBusVoltage() * getTurnOutput();
      inputs.turnPositionRad = getTurnEncoderPosition();

   }

   public void setDriveEncoderPosition(double position) {
      drivingEncoder.setPosition(position);
   };

   public double getDriveEncoderPosition() {
      return drivingEncoder.getPosition();
   };

   public void setDesiredDriveSpeedMPS(double speed) {
      drivingPIDController.setReference(speed, ControlType.kVelocity);
   };

   public double getDriveEncoderSpeedMPS() {
      return drivingEncoder.getVelocity();
   };

   public double getTurnEncoderPosition() {
      return turningEncoder.getPosition();
   };

   public void setDesiredTurnAngle(double angle) {
      turningPIDController.setReference(angle, ControlType.kPosition);
   };

   public double getDriveBusVoltage() {
      return drivingSparkMax.getBusVoltage();
   }

   public double getDriveOutput() {
      return drivingSparkMax.getAppliedOutput();
   }

   public double getTurnBusVoltage() {
      return turningSparkMax.getBusVoltage();
   }

   public double getTurnOutput() {
      return turningSparkMax.getAppliedOutput();
   }

   public String getName() {
      return name;
   }

   public double getChassisAngularOffset() {
      return chassisAngularOffset;
   }

   private int check(REVLibError err) {
      return err == REVLibError.kOk ? 0 : 1;
   }

}
