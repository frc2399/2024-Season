// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.gyro.GyroIO;

public class DriveSubsystem extends SubsystemBase {

  // correction PID
  private double DRIVE_P = 1.1;
  private double DRIVE_D = 0.05;

  private PIDController drivePIDController = new PIDController(DRIVE_P, 0, DRIVE_D);

  // debouncer for turning
  private double ROTATION_DEBOUNCE_TIME = 0.5;
  private Debouncer rotationDebouncer = new Debouncer(ROTATION_DEBOUNCE_TIME);

  // Odometry
  private SwerveDrivePoseEstimator poseEstimator;

  // swerve modules
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule rearLeft;
  private SwerveModule rearRight;

  private double DESIRED_ANGLE = 0;

  private GyroIO m_gyro;

  // Angular offsets of the modules relative to the chassis in radians
  public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
  public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
  public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
  public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

  // THIS IS 13 ON COMP BOT
  public static final int DRIVING_MOTOR_PINION_TEETH = 14;

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of the steering motor in the MAXSwerve Module.
  public static final boolean TURNING_ENCODER_INVERTED = true;
  public static final boolean DRIVING_ENCODER_INVERTED = false;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = Constants.NeoMotorConstants.FREE_SPEED_RPM / 60;
  public static final double WHEEL_DIAMETER_METERS = (3.0 * 0.0254);
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  // teeth on the bevel pinion
  // This is also the gear ratio (14T)
  public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
  public static final double TURNING_MOTOR_REDUCTION = 9424d / 203;
  public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
      / DRIVING_MOTOR_REDUCTION;

  public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVING_MOTOR_REDUCTION / (260.0 / 254); // meters
  public static final double DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60; // meters per
                                                                                                     // second

  public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
  public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

  public static final double DRIVING_P = 0.2;
  public static final double DRIVING_I = 0;
  public static final double DRIVING_D = 0;
  public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
  public static final double DRIVING_MIN_OUTPUT = -1;
  public static final double DRIVING_MAX_OUTPUT = 1;

  public static final double TURNING_P = 1.0;
  public static final double TURNING_1 = 0;
  public static final double TURNING_D = 0.001;
  public static final double TURNING_FF = 0;
  public static final double TURNING_MIN_OUTPUT = -1;
  public static final double TURNING_MAX_OUTPUT = 1;

  public static final CANSparkMax.IdleMode drivingMotorIdleMode = CANSparkMax.IdleMode.kBrake;
  public static final CANSparkMax.IdleMode turningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

  public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
  public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

  // Driving Parameters
  public static final double MAX_SPEED_METERS_PER_SECOND_TELEOP = 4.8;
  public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

  // Chassis configuration
  // Distance between centers of right and left wheels on robot
  public static final double TRACK_WIDTH = Units.inchesToMeters(26 - 2 * 1.75);
  // Distance between front and back wheels on robot
  public static final double WHEEL_BASE = Units.inchesToMeters(26 - 2 * 1.75);
  public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
  public static final Translation2d REAR_LEFT_OFFSET = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
  public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
  public static final Translation2d REAR_RIGHT_OFFSET = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT_OFFSET,
      FRONT_RIGHT_OFFSET,
      REAR_LEFT_OFFSET,
      REAR_RIGHT_OFFSET);

  // auto constants
  public static final double MAX_SPEED_METERS_PER_SECOND_AUTO = 3.0;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

  public static final double P_X_CONTROLLER = 1;
  public static final double P_Y_CONTROLLER = 1;
  public static final double P_THETA_CONTROLLER = 1;

  private final Field2d field2d = new Field2d();
  private FieldObject2d frontLeftField2dModule = field2d.getObject("front left module");
  private FieldObject2d rearLeftField2dModule = field2d.getObject("rear left module");
  private FieldObject2d frontRightField2dModule = field2d.getObject("front right module");
  private FieldObject2d rearRightField2dModule = field2d.getObject("rear right module");

  private ChassisSpeeds relativeRobotSpeeds;

  public Rotation2d lastAngle = new Rotation2d();

  StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Swerve/Current Modules States", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> swerveModuleDesiredStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Swerve/Desired Modules States", SwerveModuleState.struct).publish();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft,
      SwerveModule rearRight, GyroIO gyro) {
    this.m_gyro = gyro;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    SmartDashboard.putData(field2d);

    poseEstimator = new SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition() },
        new Pose2d(0, 0, new Rotation2d(0, 0))); // TODO: make these constants in the constants file rather than
                                                 // free-floating numbers

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::setRobotRelativeSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0), // Translation
            new PIDConstants(5, 0, 0), // Rotation
            MAX_SPEED_METERS_PER_SECOND_AUTO,
            0.385, /* Distance from furthest module to robot center in meters */
            new ReplanningConfig()),

        () -> {
          // Basically flips the path for path planner depending on alliance(Origin is
          // Blue Alliance)

          var alliance = DriverStation.getAlliance();

          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },

        this);

    configurePathPlannerLogging();
  }

  @Override
  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    SmartDashboard.putNumber("left front distance (meters)", frontLeft.getDriveEncoderPosition());
    SmartDashboard.putNumber("drive/gyro angle(degrees)", Math.toDegrees(m_gyro.getYaw()));
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromRadians(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        }); // TODO: look at updating without time

    var pose = getPose();
    SmartDashboard.putNumber("robot pose theta", pose.getRotation().getDegrees());
    field2d.setRobotPose(pose);

    frontLeftField2dModule.setPose(pose.transformBy(new Transform2d(
        FRONT_LEFT_OFFSET,
        new Rotation2d(frontLeft.getTurnEncoderPosition()))));

    rearLeftField2dModule.setPose(pose.transformBy(new Transform2d(
        REAR_LEFT_OFFSET,
        new Rotation2d(rearLeft.getTurnEncoderPosition()))));

    frontRightField2dModule.setPose(pose.transformBy(new Transform2d(
        FRONT_RIGHT_OFFSET,
        new Rotation2d(frontRight.getTurnEncoderPosition()))));

    rearRightField2dModule.setPose(pose.transformBy(new Transform2d(
        REAR_RIGHT_OFFSET,
        new Rotation2d(rearRight.getTurnEncoderPosition()))));

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState(),
    };
    swerveModuleStatePublisher.set(swerveModuleStates);

    if (Robot.isSimulation()) {
      double angleChange = DRIVE_KINEMATICS
          .toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond
          * (1 / Constants.CodeConstants.MAIN_LOOP_FREQUENCY);
      lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
      m_gyro.setYaw(lastAngle.getRadians());
    }
  }

  /** Returns the currently-estimated pose of the robot. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the odometry to the specified pose. */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromRadians(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotRate       Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotRate, boolean fieldRelative) {

    double newRotRate = 0;
    double currentAngle = (m_gyro.getYaw());
    double r = Math.pow(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)), 3);
    double polarAngle = Math.atan2(ySpeed, xSpeed);
    double polarXSpeed = r * Math.cos(polarAngle);
    double polarYSpeed = r * Math.sin(polarAngle);

    // //Account for edge case when gyro resets
    if (currentAngle == 0) {
      DESIRED_ANGLE = 0;
    }

    // Debouncer ensures that there is no back-correction immediately after turning
    // Deadband for small movements - they are so slight they do not need correction
    // and correction causes robot to spasm
    if (rotationDebouncer.calculate(rotRate == 0)
        && (Math.abs(polarXSpeed) >= 0.075 || Math.abs(polarYSpeed) != 0.075)) {
      newRotRate = newRotRate + drivePIDController.calculate(currentAngle, DESIRED_ANGLE);
    } else {
      newRotRate = rotRate;
      DESIRED_ANGLE = currentAngle;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = polarXSpeed * MAX_SPEED_METERS_PER_SECOND_TELEOP;
    double ySpeedDelivered = polarYSpeed * MAX_SPEED_METERS_PER_SECOND_TELEOP;
    double rotRateDelivered = newRotRate * MAX_ANGULAR_SPEED;

    if (fieldRelative) {
      relativeRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered,
          Rotation2d.fromRadians(m_gyro.getYaw()));
    } else {
      relativeRobotSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered);
    }

    SmartDashboard.putNumber("Swerve/velocity",
        Math.sqrt(
            Math.pow(relativeRobotSpeeds.vxMetersPerSecond, 2) + Math.pow(relativeRobotSpeeds.vyMetersPerSecond, 2)));

    var swerveModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(relativeRobotSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MAX_SPEED_METERS_PER_SECOND_TELEOP);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    swerveModuleDesiredStatePublisher.set(swerveModuleStates);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DRIVE_KINEMATICS.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
        rearLeft.getState(), rearRight.getState());
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, .02);
    var swerveModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MAX_SPEED_METERS_PER_SECOND_TELEOP);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    swerveModuleDesiredStatePublisher.set(swerveModuleStates);

  }

  private void configurePathPlannerLogging() {
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field2d.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field2d.getObject("ROBOT target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field2d.getObject("ROBOT path").setPoses(poses);
    });
  }
}
