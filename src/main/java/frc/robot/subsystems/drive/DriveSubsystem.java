// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.subsystems.gyro.GyroIO;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {

  // The gyro sensor
  // private final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);

  // Odometry
  private SwerveDrivePoseEstimator poseEstimator;

  // swerve modules
  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_rearLeft;
  private SwerveModule m_rearRight;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotationRate = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double desiredAngle = 0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private LinearFilter derivativeCalculator = LinearFilter.backwardFiniteDifference(1, 2, 0.02);
  private double pitchRate;
  private GyroIO m_gyro;

  private final Field2d field2d = new Field2d();
  private FieldObject2d frontLeftField2dModule = field2d.getObject("front left module");
  private FieldObject2d rearLeftField2dModule = field2d.getObject("rear left module");
  private FieldObject2d frontRightField2dModule = field2d.getObject("front right module");
  private FieldObject2d rearRightField2dModule = field2d.getObject("rear right module");

  private ChassisSpeeds relativeRobotSpeeds; 

  public Rotation2d lastAngle = new Rotation2d();

  StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/Swerve/Current Modules States", SwerveModuleState.struct).publish(); 

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(SwerveModule m_frontLeft, SwerveModule m_frontRight, SwerveModule m_rearLeft,
      SwerveModule m_rearRight, GyroIO m_gyro) {
    this.m_gyro = m_gyro;
    this.m_frontLeft = m_frontLeft;
    this.m_frontRight = m_frontRight;
    this.m_rearLeft = m_rearLeft;
    this.m_rearRight = m_rearRight;

    SmartDashboard.putData(field2d);

    poseEstimator = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics, new Rotation2d(m_gyro.getYaw()), getModulePositions(), 
        new Pose2d());


    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::setRobotRelativeSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0), // Translation
            new PIDConstants(0.975, 0, 0), // Rotation
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
  }

  @Override
  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromRadians(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    SmartDashboard.putNumber("periodic front left position", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("periodic front right position", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("periodic rear left position", m_rearLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("periodic rear right position", m_rearRight.getPosition().distanceMeters);


    SmartDashboard.putNumber("periodic front left angle", m_frontLeft.getPosition().angle.getDegrees());
    // Gyro log (spain without the a followed by spain without the s)
    SmartDashboard.putNumber("Gyro angle", m_gyro.getYaw() % 360);
    // SmartDashboard.putNumber("Gyro pitch", Gyro.pitch % 360);
    // SmartDashboard.putNumber("Gyro roll", Gyro.roll % 360);
    // pitchRate = derivativeCalculator.calculate(getGyroPitch());
    // Drive input log
    SmartDashboard.putNumber("Right Front Drive Input", m_frontRight.getDriveBusVoltage());
    SmartDashboard.putNumber("Left Front Drive Input", m_frontLeft.getDriveBusVoltage());
    SmartDashboard.putNumber("Right Rear Drive Input", m_rearRight.getDriveBusVoltage());
    SmartDashboard.putNumber("Left Rear Drive Input", m_rearLeft.getDriveBusVoltage());
    // Drive output log
    SmartDashboard.putNumber("Right Front Drive Output", m_frontRight.getDriveOutput());
    SmartDashboard.putNumber("Left Front Drive Output", m_frontLeft.getDriveOutput());
    SmartDashboard.putNumber("Right Rear Drive Output", m_rearRight.getDriveOutput());
    SmartDashboard.putNumber("Left Rear Drive Output", m_rearLeft.getDriveOutput());

    // updates inputs for each module
    m_frontLeft.updateInputs();
    m_rearLeft.updateInputs();
    m_frontRight.updateInputs();
    m_rearRight.updateInputs();

    field2d.setRobotPose(getPose());
    SmartDashboard.putNumber("robot pose", getPose().getRotation().getDegrees());

    frontLeftField2dModule.setPose(getPose().transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_LEFT_OFFSET,
        new Rotation2d(m_frontLeft.getTurnEncoderPosition()))));

    rearLeftField2dModule.setPose(getPose().transformBy(new Transform2d(
        Constants.DriveConstants.REAR_LEFT_OFFSET,
        new Rotation2d(m_rearLeft.getTurnEncoderPosition()))));

    frontRightField2dModule.setPose(getPose().transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_RIGHT_OFFSET,
        new Rotation2d(m_frontRight.getTurnEncoderPosition()))));

    rearRightField2dModule.setPose(getPose().transformBy(new Transform2d(
        Constants.DriveConstants.REAR_RIGHT_OFFSET,
        new Rotation2d(m_rearRight.getTurnEncoderPosition()))));

    
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
    swerveModuleStatePublisher.set(swerveModuleStates);

    double angleChange = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond * (1/Constants.CodeConstants.kMainLoopFrequency);
    lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
    m_gyro.setYaw(lastAngle.getRadians());
  }
  // Log empty setpoint states when disabled

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();

  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (RobotBase.isReal()) {
      poseEstimator.resetPosition(new Rotation2d(m_gyro.getYaw()), getModulePositions(), pose);
    } else {
      poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw()),
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
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rotRate, boolean fieldRelative, boolean rateLimit) {

    double newRotRate = 0;
    double xSpeedCommanded;
    double ySpeedCommanded;
    double currentAngle = (m_gyro.getYaw());

    if (currentAngle == 0) {
      desiredAngle = 0;
      newRotRate = rotRate;
    }

    else if (rotRate == 0) {
      newRotRate = 0;

      if (Math.abs(desiredAngle - currentAngle) > Math.toRadians(0.1)) {
        newRotRate = 3 * (desiredAngle - currentAngle) / (2 * Math.PI);
      }
    } else {
      newRotRate = rotRate;
      desiredAngle = currentAngle;
    }
    SmartDashboard.putNumber("desired angle", desiredAngle);
    SmartDashboard.putNumber("current angle", currentAngle);
    SmartDashboard.putNumber("rotRate", rotRate);
    SmartDashboard.putNumber("rotation rate", newRotRate);

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotationRate = m_rotRateLimiter.calculate(newRotRate);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotationRate = newRotRate;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRateDelivered = m_currentRotationRate * DriveConstants.kMaxAngularSpeed;


    relativeRobotSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered,
                Rotation2d.fromRadians(m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered);

    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(relativeRobotSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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

  public void setZero() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromRadians(m_gyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  // public double getGyroPitch() {
  // return -Gyro.pitch;
  // }

  // public double getGyroPitchRate()
  // {
  // return pitchRate;
  // }

  // Returns the distance and angle of each module
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontLeft.getPosition();
    positions[1] = m_frontRight.getPosition();
    positions[2] = m_rearLeft.getPosition();
    positions[3] = m_rearRight.getPosition();
    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(), m_rearRight.getState());
  }


  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);
  }
}
