// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class CodeConstants {
    public static final double kMainLoopFrequency = 50; // Hz
  }

  public static final int NEO550_CURRENT_LIMIT = 20;
  public static final int NEO_CURRENT_LIMIT = 50;
  public static final int NEO_MAX_SPEED_RPM = 5676;
  // MPS = (GearRatio * 2πr * RPM) / 60
  public static final double NEO550_MAX_SPEED_RPM = 11000;

  public static final class DriveConstants {
    public static final int kGyroCanId = 3;
    // Driving Parameters
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(26 - 2 * 1.75);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26 - 2 * 1.75);
    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d REAR_LEFT_OFFSET = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d REAR_RIGHT_OFFSET = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        FRONT_LEFT_OFFSET,
        FRONT_RIGHT_OFFSET,
        REAR_LEFT_OFFSET,
        REAR_RIGHT_OFFSET);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 21;
    public static final int kFrontRightDrivingCanId = 31;
    public static final int kRearRightDrivingCanId = 41;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontRightTurningCanId = 32;
    public static final int kRearRightTurningCanId = 42;

  }

  public static final class SwerveModuleConstants {
    //THIS IS 13 ON COMP BOT
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;
    public static final boolean kDrivingEncoderInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // This is also the gear ratio (14T)
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kTurningMotorReduction = 9424d / 203;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.2;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.001;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final CANSparkMax.IdleMode kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class IntakeConstants {

    public static final int LEFT_CENTERING_MOTOR_ID = 5;
    public static final int RIGHT_CENTERING_MOTOR_ID = 4;
    public static final int INTAKE_CENTERING_ID = 2; 
    public static final double INTAKING_SPEED = 0.8;
    public static final double OUTTAKING_SPEED = -0.6;
  }

  public static final class ShooterConstants {

    public static final int SHOOT_LOW_MOTOR_ID = 7;
    public static final int SHOOT_HIGH_MOTOR_ID = 8;
    public static final double SPEAKER_SPEED = 0.8;
    public static final double AMP_SPEED = 0.3;
    public static final double FAR_AWAY_SPEED = 1;
    public static final double SUBWOOFER_SPEED = 0.6;
    public static final double SHOOTER_FEEDFORWRD = 0.03;
    public static final double SHOOTER_PVALUE = 0.01;
    public static final double SHOOT_MAX_SPEED_RPS = NEO_MAX_SPEED_RPM / 60;
  }

  public static final class IndexerConstants {

    public static final int INDEXER_MOTOR_ID = 6; 
    public static final int INDEXER_SENSOR_CHANNEL_TOP = 0; 
    public static final int INDEXER_SENSOR_CHANNEL_BOTTOM = 1;
    public static final double INDEXER_IN_SPEED = 0.8;
    public static final double INDEXER_OUTTAKING_SPEED = -0.6;
  }

  public static final class ArmConstants {

    public static final int ARM_MOTOR_ID_LEFT = 9;
    public static final int ARM_MOTOR_ID_RIGHT = 10;

    // arm angles for intaking and shooting in different positions, in radians
    // intake angle same as initial offset
    public static final double INTAKE_ANGLE = Units.degreesToRadians(14);
    public static final double SPEAKER_SUBWOOFER_ANGLE = Units.degreesToRadians(14);;
    public static final double SPEAKER_PODIUM_ANGLE = -0.06;
    public static final double AMP_ANGLE = Units.degreesToRadians(90);

    //1 4-1 gearbox, 2 3-1 gearboxes, then a 4-1 reduction from the sprocket/chain; 4^2 * 3^2 = 144
    public static final double RADIANS_PER_REVOLUTION = 2 * Math.PI / 144;
    //absolute encoder only needs the sprocket/chain reduction, since it comes after the gearboxes
    public static final double ABSOLUTE_RADIANS_PER_REVOLUTION = 2 * Math.PI / 4;

    public static final double INITIAL_OFFSET = Units.degreesToRadians(14);

    // can be 2 degrees off from goal setpoints in auton and still considered at goal
    public static final double ANGLE_TOLERANCE_AUTON = Units.degreesToRadians(2);

    //sim constants
     // arm min and max angles in radians
     public static final double MAX_ARM_ANGLE = Math.PI / 4 * 3;
     // initial offset -5 degrees
     public static final double MIN_ARM_ANGLE = 0;
     // arm mass in kg
     public static final double ARM_MASS = 2.72155;
     // arm length in meters
     public static final double ARM_LENGTH = 0.65;

  }

  public static final class ClimberConstants {

    public static final int LEFT_CLIMBER_MOTOR_ID = 13;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 14;
    public static final double CLIMBER_SPEED = 0.5;

    public static final double MAX_HEIGHT = 0.15;
    public static final double MIN_HEIGHT = 0;

    public static final double VELOCITY_THRESHHOLD = 20;

    public static final double CLIMBER_RADIUS = 0.375;

    public static final double SETPOINT_RANGE = 0.05;

    public static final double ENCODER_VELOCITY_MPS = (2 * (Math.PI) * 0.375) / 60 / 12;
    public static final double ENCODER_METERS = (2 * (Math.PI) * 0.375 / 12);

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

    public static final class LEDConstants {
    public static final int RED_CHANNEL = 5;
    public static final int GREEN_CHANNEL = 6;
    public static final int BLUE_CHANNEL = 7;
    public static final int WHITE_CHANNEL = 8;
    public static final int[] blue2399 = { 0, 100, 255 };
    public static final int[] pink2399 = { 255, 50, 200 };
    public static final int[] isIntookedGreen = { 112, 243, 121 };
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d camToRobot = new Transform3d(
        new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(14.25), Units.inchesToMeters(0)),
        new Rotation3d(0.0, Units.degreesToRadians(15.0), 0.0));
        // TODO: measure camera height
    //these need to be changed on Comp Bot
    public static final Transform2d camToRobot2d = new Transform2d(
      Units.inchesToMeters(5),
      Units.inchesToMeters(14.25),
      new Rotation2d(0));
    public static final double xOffsetToRobot = Units.inchesToMeters(5);
    public static final double yOffsetToRobot = Units.inchesToMeters(14.25);
    public static final double zOffsetToRobot = Units.inchesToMeters(0);

     //5.33E-03*x + 0.206 - https://docs.google.com/spreadsheets/d/1TCEiHto6ypUku9VXPN79PGwONyrlhI2SbMsfn337yTw/edit#gid=0
    // inverse tan of function above to get angle
    //all in inches except for eightyModelRange
    public static final double eightyModelSlope = 0.00533;
    public static final double eightyModelIntercept = 0.206;
    public static final double eightyModelRange = Units.feetToMeters(12);
    public static final double hundredModelSlope = 0.00533;
    public static final double hundredModelIntercept = 0.206;

  }

}
