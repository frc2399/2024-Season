// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.gyro.GyroIO;

public class DriveSubsystem extends SubsystemBase {

  // vision
  private VisionIO visionIO;
  private double MAX_VISION_UPDATE_SPEED_MPS = 0.5 * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
  private double velocityXMPS;
  private double velocityYMPS;
  private double velocityMPS;
  private Pose2d visionEstimatedPose;
  private Pose2d robotPose;
  private Pose2d speakerPose;

  // apriltags
  public int speakerID;
  public int ampID;
  public boolean isAligned = false;

  // PID for the speaker-aiming method
  final double ANGULAR_P = 1.0;
  final double ANGULAR_D = 0.015;
  PIDController keepPointedController = new PIDController(
      ANGULAR_P, 0, ANGULAR_D);
  final static double ANGLE_TO_SPEAKER_ROT_RATE_TOLERANCE = 0.01;

  Optional<EstimatedRobotPose> possiblePose;

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

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotationRate = 0.0;

  private double desiredAngle = 0;

  private GyroIO gyroIO;

  private final Field2d field2d = new Field2d();
  private final Field2d visionField = new Field2d();
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
      SwerveModule rearRight, GyroIO gyro, VisionIO vision) {
    this.visionIO = vision;
    this.gyroIO = gyro;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    SmartDashboard.putData(field2d);

    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
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
            AutoConstants.kMaxSpeedMetersPerSecond,
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
    SmartDashboard.putNumber("drive/gyro angle(degrees)", Math.toDegrees(gyroIO.getYaw()));
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromRadians(gyroIO.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    Pose2d pose = getPose();

    velocityXMPS = getRobotRelativeSpeeds().vxMetersPerSecond;
    velocityYMPS = getRobotRelativeSpeeds().vyMetersPerSecond;
    velocityMPS = Math.sqrt((Math.pow(velocityXMPS, 2) + Math.pow(velocityYMPS, 2)));

    robotPose = poseEstimator.getEstimatedPosition();

    if (velocityMPS <= MAX_VISION_UPDATE_SPEED_MPS) {
      possiblePose = visionIO.getVisionPose();
      // makes sure that there is a new pose and that there are targets before getting
      // a robot pose
      if (possiblePose.isPresent()) {
        visionEstimatedPose = possiblePose.get().estimatedPose.toPose2d();
        double distanceToTag = Math.hypot(visionEstimatedPose.getX(), visionEstimatedPose.getY());
        poseEstimator.addVisionMeasurement(visionEstimatedPose, Timer.getFPGATimestamp(),
            VecBuilder.fill(distanceToTag / 2, distanceToTag / 2, 100));
      }
    }

    SmartDashboard.putNumber("robot pose theta", pose.getRotation().getDegrees());
    field2d.setRobotPose(pose);

    frontLeftField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_LEFT_OFFSET,
        new Rotation2d(frontLeft.getTurnEncoderPosition()))));

    rearLeftField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.REAR_LEFT_OFFSET,
        new Rotation2d(rearLeft.getTurnEncoderPosition()))));

    frontRightField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_RIGHT_OFFSET,
        new Rotation2d(frontRight.getTurnEncoderPosition()))));

    rearRightField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.REAR_RIGHT_OFFSET,
        new Rotation2d(rearRight.getTurnEncoderPosition()))));

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState(),
    };
    swerveModuleStatePublisher.set(swerveModuleStates);

    if (Robot.isSimulation()) {
      double angleChange = Constants.DriveConstants.kDriveKinematics
          .toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond * (1 / Constants.CodeConstants.kMainLoopFrequency);
      lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
      gyroIO.setYaw(lastAngle.getRadians());
    }
  }

  /** Returns the currently-estimated pose of the robot. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();

  }

  /** Resets the odometry to the specified pose. */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromRadians(gyroIO.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
    // frontLeft.setDriveEncoderPosition(poseEstimator.getEstimatedPosition().);
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
  public void drive(double xSpeed, double ySpeed, double rotRate, boolean fieldRelative,
      boolean alignToSpeakerWithVision) {

    double newRotRate = 0;
    double currentAngle = (gyroIO.getYaw());
    double r = Math.pow(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)), 3);
    double polarAngle = Math.atan2(ySpeed, xSpeed);
    double polarXSpeed = r * Math.cos(polarAngle);
    double polarYSpeed = r * Math.sin(polarAngle);

    // //Account for edge case when gyro resets
    if (currentAngle == 0) {
      desiredAngle = 0;
    }

    // Debouncer ensures that there is no back-correction immediately after turning
    // Deadband for small movements - they are so slight they do not need correction
    // and correction causes robot to spasm

    if (alignToSpeakerWithVision) {
      newRotRate = getAlignToSpeakerRotRate(currentAngle);
    } else {
      newRotRate = getHeadingCorrectionRotRate(currentAngle, rotRate, polarXSpeed, polarYSpeed);
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = polarXSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = polarYSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotRateDelivered = newRotRate * DriveConstants.MAX_ANGULAR_SPEED;

    if (fieldRelative) {
      relativeRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered,
          Rotation2d.fromRadians(gyroIO.getYaw()));
    } else {
      relativeRobotSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered);
    }

    SmartDashboard.putNumber("Swerve/velocity",
        Math.sqrt(
            Math.pow(relativeRobotSpeeds.vxMetersPerSecond, 2) + Math.pow(relativeRobotSpeeds.vyMetersPerSecond, 2)));

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(relativeRobotSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
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
    return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
        rearLeft.getState(), rearRight.getState());
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, .02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
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

  // public Command driveCommand(double xSpeed, double ySpeed, double rotRate,
  // boolean fieldRelative,
  // boolean alignToSpeakerWithVision) {
  // return this.run(() -> drive(xSpeed, ySpeed, rotRate, fieldRelative,
  // alignToSpeakerWithVision));
  // }

  // assigns aprilTags based on alliance
  public void setAprilTagIDsAndLocations(Optional<Alliance> ally) {
    if (ally.get() == Alliance.Red) {
      speakerID = 4;
      ampID = 5;
    } else {
      speakerID = 7;
      ampID = 6;
    }
    speakerPose = VisionConstants.KFIELDLAYOUT.getTagPose(speakerID).get().toPose2d();
  }

  public Pose2d getSpeakerPose() {
    return speakerPose;
  }

  private double getAlignToSpeakerRotRate(double currentAngle) {
    // adding pi to account for the 180 degree rotation - without it, robot wants to
    // point its front at the speaker, and we want the back pointed directly at
    // speaker
    double angleToSpeaker = PhotonUtils.getYawToPose(robotPose,
        speakerPose).getRadians() + Math.PI;
    // allows the robot to turn in the most efficient direction. for example, if the
    // robot is six degrees off the target and needs to rotate counterclockwise,
    // due to other needed corrections, this causes the robot to spin 354 degrees
    // CLOCKWISE. subtracting 2pi from values over pi radians makesthese values
    // negative and thus the robot rotates in the correct direction.
    if (angleToSpeaker > Math.PI) {
      angleToSpeaker -= (2 * Math.PI);
    }
    double rotRate = -keepPointedController.calculate((angleToSpeaker % (2 * Math.PI)), 0);
    desiredAngle = currentAngle;

    // deadband to prevent overresponsiveness to small errors produced by pose
    // estimator noise
    if (Math.abs(rotRate) < ANGLE_TO_SPEAKER_ROT_RATE_TOLERANCE) {
      rotRate = 0;
    }
    return rotRate;
  }

  private double getHeadingCorrectionRotRate(double currentAngle, double rotRate, double polarXSpeed,
      double polarYSpeed) {
    double newRotRate = 0;
    if (rotationDebouncer.calculate(rotRate == 0)
        && (Math.abs(polarXSpeed) >= 0.075 || Math.abs(polarYSpeed) != 0.075)) {
      newRotRate = newRotRate + drivePIDController.calculate(currentAngle, desiredAngle);
    } else {
      newRotRate = rotRate;
      desiredAngle = currentAngle;
    }
    return newRotRate;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }
}
