// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
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
  private SwerveDrivePoseEstimator m_poseEstimator;

  // swerve modules
  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_rearLeft;
  private SwerveModule m_rearRight;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotationRate = 0.0;

  private double desiredAngle = 0;

  private GyroIO m_gyro;

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
  public DriveSubsystem(SwerveModule m_frontLeft, SwerveModule m_frontRight, SwerveModule m_rearLeft,
      SwerveModule m_rearRight, GyroIO m_gyro) {
    this.m_gyro = m_gyro;
    this.m_frontLeft = m_frontLeft;
    this.m_frontRight = m_frontRight;
    this.m_rearLeft = m_rearLeft;
    this.m_rearRight = m_rearRight;

    SmartDashboard.putData(field2d);

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() },
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
    SmartDashboard.putNumber("left front distance (meters)", m_frontLeft.getDriveEncoderPosition());
    SmartDashboard.putNumber("drive/gyro angle(degrees)", Math.toDegrees(m_gyro.getYaw()));
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromRadians(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }); // TODO: look at updating without time

    var pose = getPose();
    SmartDashboard.putNumber("robot pose theta", pose.getRotation().getDegrees());
    field2d.setRobotPose(pose);

    frontLeftField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_LEFT_OFFSET,
        new Rotation2d(m_frontLeft.getTurnEncoderPosition()))));

    rearLeftField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.REAR_LEFT_OFFSET,
        new Rotation2d(m_rearLeft.getTurnEncoderPosition()))));

    frontRightField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_RIGHT_OFFSET,
        new Rotation2d(m_frontRight.getTurnEncoderPosition()))));

    rearRightField2dModule.setPose(pose.transformBy(new Transform2d(
        Constants.DriveConstants.REAR_RIGHT_OFFSET,
        new Rotation2d(m_rearRight.getTurnEncoderPosition()))));

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState(),
    };
    swerveModuleStatePublisher.set(swerveModuleStates);

    if (Robot.isSimulation()) {
      double angleChange = Constants.DriveConstants.kDriveKinematics
          .toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond * (1 / Constants.CodeConstants.kMainLoopFrequency);
      lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
      m_gyro.setYaw(lastAngle.getRadians());
    }
  }

  /** Returns the currently-estimated pose of the robot. */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();

  }

  /** Resets the odometry to the specified pose. */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromRadians(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
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
    double xSpeedCommanded;
    double ySpeedCommanded;
    double currentAngle = (m_gyro.getYaw());

    // //Account for edge case when gyro resets
    if (currentAngle == 0) {
      desiredAngle = 0;
    }

    // Debouncer ensures that there is no back-correction immediately after turning
    // Deadband for small movements - they are so slight they do not need correction
    // and correction causes robot to spasm
    if (rotationDebouncer.calculate(rotRate == 0) && (Math.abs(xSpeed) >= 0.075 || Math.abs(ySpeed) != 0.075)) {
      newRotRate = newRotRate + drivePIDController.calculate(currentAngle, desiredAngle);
    } else {
      newRotRate = rotRate;
      desiredAngle = currentAngle;
    }

    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotationRate = newRotRate;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRateDelivered = m_currentRotationRate * DriveConstants.kMaxAngularSpeed;

    if (fieldRelative) {
      relativeRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered,
          Rotation2d.fromRadians(m_gyro.getYaw()));
    } else {
      relativeRobotSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered);
    }

    SmartDashboard.putNumber("Swerve/ velocity", relativeRobotSpeeds.vxMetersPerSecond);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(relativeRobotSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    swerveModuleDesiredStatePublisher.set(swerveModuleStates);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(), m_rearRight.getState());
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, .02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

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
