// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int NEO550_CURRENT_LIMIT = 20;
  public static final int NEO_CURRENT_LIMIT = 50;
  public static final int NEO_MAX_SPEED_RPM = 5676; 
  //MPS = (GearRatio * 2πr * RPM) / 60
  public static final double NEO550_MAX_SPEED_RPM = 11000;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2 + 3.6128315516282616;
    // public static final double kFrontRightChassisAngularOffset = 0 + 5.855928706291374;
    // public static final double kBackLeftChassisAngularOffset = Math.PI + 3.248406803811846;
    // public static final double kBackRightChassisAngularOffset = Math.PI / 2 + 2.2556635252774715;

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;


    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 21;
    public static final int kFrontRightDrivingCanId = 31;
    public static final int kRearRightDrivingCanId = 41;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontRightTurningCanId = 32;
    public static final int kRearRightTurningCanId = 42;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;
    //testing lol
    public static final boolean kDrivingEncoderInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    //This is also the gear ratio (14T)
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
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

    public static final double kTurningP = 2.5;
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

    public static final int LEFT_CENTERING_MOTOR_ID = 1;
    public static final int RIGHT_CENTERING_MOTOR_ID = 2;
    public static final int INTAKE_CENTERING_ID = 5; //RANDOM NUMBER --> CHANGE!!!!
    public static final double INTAKE_SLEW_RATE = 10;
    public static final int INTAKE_SENSOR_CHANNEL = 0;
    public static final double INTAKING_SPEED = 0.8;
    public static final double OUTTAKING_SPEED = -0.6;
  }

  public static final class ShooterConstants {

    public static final int SHOOT_LOW_MOTOR_ID = 4;
    public static final int SHOOT_HIGH_MOTOR_ID = 3;
    public static final double speakerSpeed = 0.8;
    public static final double ampSpeed = 0.3;
    public static final double NEO_MAX_SPEED_MPS = 2 * Math.PI * 0.0508 * NEO_MAX_SPEED_RPM * (1 / 60);
  }

  public static final class IndexerConstants {

    public static final int INDEXER_MOTOR_ID = 5; //I put 5 at random. Find actual motor ID. 
    public static final double INDEXER_SLEW_RATE = 10;
    public static final int INDEXER_SENSOR_CHANNEL = 1; //change as necessary
    public static final double INDEXER_IN_SPEED = 0.8; 
    public static final double INDEXER_OUTTAKING_SPEED = -0.6;
  }

  public static final class ArmConstants {

    public static final int ARM_MOTOR_ID_LEFT = 6;
    public static final int ARM_MOTOR_ID_RIGHT = 7;

    // arm min and max angles in radians
    public static final double MAX_ARM_ANGLE = Math.PI / 4 * 3;
    //initial offset -5 degrees
    public static final double MIN_ARM_ANGLE = 0;
    // arm mass in kg
    public static final double ARM_MASS = 2.72155;
    // arm length in meters
    public static final double ARM_LENGTH = 0.65;

    // arm angles for intaking and shooting in different positions, in radians
    //intake angle same as initial offset
    public static final double INTAKE_ANGLE = 0;
    public static final double SPEAKER_SUBWOOFER_ANGLE = -0.66;
    public static final double SPEAKER_PODIUM_ANGLE = -0.06;
    public static final double AMP_ANGLE = -0.25;

    public static final double TURTLE_ANGLE = 0.5;

    // 1 4-1 gearbox, 2 3-1 gearboxes, then a 4-1 reduction from the sprocket/chain; 4^2 * 3^2 = 144
    public static final double RADIANS_PER_REVOLUTION = 2 * Math.PI / 144;
    // initial offset is 0.711 + (0.287) - (0.308)
    
    public static final double INITIAL_OFFSET = 0.660;

    //can be 2 degrees off from goal setpoints and still considered at goal; made higher so arm.atGoal() in placeConeOnNode cmd will execute in auton
    public static final double ANGLE_TOLERANCE_AUTON = Units.degreesToRadians(2);

    public static final double SPEAKER_SUBWOOFER_STRAIGHT_ANGLE = 0;

    public static final double SPEAKER_SUBWOOFER_SIDE_ANGLE = 0;

    }

  public static final class ClimberConstants {

    public static final int LEFT_CLIMBER_MOTOR_ID = 2;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 1;
    public static final double CLIMBER_SPEED = 0.5;

    public static final double MAX_HEIGHT = 0.4;
    public static final double MIN_HEIGHT = 0;

    public static final int CLIMBER_SLEW = 5;

    public static final int EXTEND_PISTON = 4;
    public static final int RETRACT_PISTON = 5;

    public static final double VELOCITY_THRESHHOLD = 20;

    public static final double CLIMBER_RADIUS = 0.375; 

    public static final double SETPOINT_RANGE = 0.05; 

    public static final double ENCODER_VELOCITY_MPS = (2 * (Math.PI) * 0.375) / 60;
    public static final double ENCODER_METERS = (2 * (Math.PI) * 0.375); 


  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
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

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
