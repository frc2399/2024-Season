// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.NavX.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;




public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotationRate = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double desiredAngle = 0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(Gyro.yaw),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private LinearFilter derivativeCalculator = LinearFilter.backwardFiniteDifference(1, 2, 0.02);
  private double pitchRate;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    // m_odometry.update(m_gyro.getRotation2d(),
    // m_leftEncoder.getDistance(),
    // m_rightEncoder.getDistance());
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    //Gyro log (spain without the a followed by spain without the s)
    SmartDashboard.putNumber("Gyro angle", Gyro.yaw % 360);
    SmartDashboard.putNumber("Gyro pitch", Gyro.pitch % 360);
    SmartDashboard.putNumber("Gyro roll", Gyro.roll % 360);
    pitchRate = derivativeCalculator.calculate(getGyroPitch());
    //Drive input log
    SmartDashboard.putNumber("Right Front Drive Input", m_frontRight.getDriveVolts());
    SmartDashboard.putNumber("Left Front Drive Input", m_frontLeft.getDriveVolts());
    SmartDashboard.putNumber("Right Rear Drive Input", m_rearRight.getDriveVolts());
    SmartDashboard.putNumber("Left Rear Drive Input", m_rearLeft.getDriveVolts());
    //Drive output log
    SmartDashboard.putNumber("Right Front Drive Output", m_frontRight.getDriveOutput());
    SmartDashboard.putNumber("Left Front Drive Output", m_frontLeft.getDriveOutput());
    SmartDashboard.putNumber("Right Rear Drive Output", m_rearRight.getDriveOutput());
    SmartDashboard.putNumber("Left Rear Drive Output", m_rearLeft.getDriveOutput());
    //Drive speed log
    SmartDashboard.putNumber("Right Front Drive Speed", m_frontRight.getDriveSpeed());
    SmartDashboard.putNumber("Left Front Drive Speed", m_frontLeft.getDriveSpeed());
    SmartDashboard.putNumber("Right Rear Drive Speed", m_rearRight.getDriveSpeed());
    SmartDashboard.putNumber("Left Rear Drive Speed", m_rearLeft.getDriveSpeed());
    //Turn speed log
    SmartDashboard.putNumber("Right Front Turn Speed", m_frontRight.getTurnAngle());
    SmartDashboard.putNumber("Left Front Turn Speed", m_frontLeft.getTurnAngle());
    SmartDashboard.putNumber("Right Rear Turn Speed", m_rearRight.getTurnAngle());
    SmartDashboard.putNumber("Left Rear Turn Speed", m_rearLeft.getTurnAngle());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(Gyro.yaw),
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
   * @param rotRate           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rotRate, boolean fieldRelative, boolean rateLimit) {
    
    double newRotRate = 0;
    double xSpeedCommanded;
    double ySpeedCommanded;
    double currentAngle = Math.toRadians(Gyro.yaw);

    if (currentAngle == 0) {
      desiredAngle = 0;
    }

    else if(rotRate == 0) {
      newRotRate = 0;
      
      if(Math.abs(desiredAngle - currentAngle) > Math.toRadians(0.1)) {
        newRotRate = 3 * (desiredAngle - currentAngle) / (2 * Math.PI);
      }
    } 
    else {
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

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered, Rotation2d.fromDegrees(Gyro.yaw))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered));
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
    return Rotation2d.fromDegrees(Gyro.yaw).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  //   return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public double getGyroPitch() {
    return -Gyro.pitch;
  }

  public double getGyroPitchRate()
  {
      return pitchRate;
  }

  public void setMotorSpeeds(double speed)
  {
    m_frontLeft.m_drivingSparkMax.set(speed);
    m_frontRight.m_drivingSparkMax.set(speed);
    m_rearLeft.m_drivingSparkMax.set(speed);
    m_rearRight.m_drivingSparkMax.set(speed);
  }
}
