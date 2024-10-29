// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.PIDUtil;

public class Arm extends ProfiledPIDSubsystem {
  /** Creates a new Arm. */
  private ArmIO armIO;

  public static double speedFromArmHeight;

  // Trapezoidal profile constants and variables
  // TODO tune max_vel and max_accel
  private static final double max_vel = 4; // rad/s (NEO specs / gear ratio, converted into rad/s ~ 4.1, give it a
                                           // slightly lower one to make it acheivable)
  private static final double max_accel = 6; // rad/s/s
  private static final Constraints constraints = new Constraints(max_vel, max_accel);
  private static double gravityCompensation = 0.025;
  private static double feedForward = 1 / max_vel;
  private static double kpPos = 3.0;
  private static double kd = 0.01;

  private static final double SPEAKER_SUBWOOFER_ANGLE = Units.degreesToRadians(14);;

  private static final double VISION_ARM_ALIGNMENT_STAY_SUBWOOF_POSITION_BOUNDARY_INCHES = (46.5);
  private static final double VISION_ARM_ALIGNMENT_EIGHTY_PERCENT_SHOOTER_SPEED_MODEL_SLOPE = 0.00346;
  private static final double VISION_ARM_ALIGNMENT_EIGHTY_PERCENT_SHOOTER_SPEED_MODEL_Y_INTERCEPT = 0.298;
  private static final double VISION_ARM_ALIGNMENT_EIGHTY_PERCENT_SHOOTER_SPEED_MODEL_RANGE = Units.feetToMeters(12);
  private static final double VISION_ARM_ALIGNMENT_HUNDRED_PERCENT_SHOOTER_SPEED_MODEL_SLOPE = 0.00346;
  private static final double VISION_ARM_ALIGNMENT_HUNDRED_PERCENT_SHOOTER_SPEED_MODEL_Y_INTERCEPT = 0.298;

  public Arm(ArmIO io) {
    super(new ProfiledPIDController(kpPos, 0, kd, constraints));
    armIO = io;
  }

  @Override
  public void periodic() {
    // Call periodic method in profile pid subsystem to prevent overriding
    super.periodic();
    armIO.periodicUpdate();
  }

  public double getEncoderPosition() {
    return armIO.getEncoderPosition();
  }

  public double getEncoderSpeed() {
    return armIO.getEncoderSpeed();
  }

  public void setSpeed(double speed) {
    armIO.setSpeed(speed);
  }

  public void setSpeedGravityCompensation(double speed) {
    setSpeed(speed + gravityCompensation * Math.cos(getEncoderPosition()));
  }

  public double getArmCurrent() {
    return armIO.getArmCurrent();
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    SmartDashboard.putNumber("arm/setpoint pos", setpoint.position);
    SmartDashboard.putNumber("arm/setpoint vel", setpoint.velocity);

    // Calculate the feedforward from the setpoint
    double speed = feedForward * setpoint.velocity;
    // accounts for gravity in speed
    speed += gravityCompensation * Math.cos(getEncoderPosition());
    // Add PID output to speed to account for error in arm
    speed += output;
    // calls set speed function in the file that does armIO.setSpeed
    setSpeed(speed);
  }

  @Override
  protected double getMeasurement() {
    return armIO.getEncoderPosition();
  }

  public double getGoal() {
    return m_controller.getGoal().position;
  }

  // Checks to see if arm is within range of the setpoints
  public boolean atGoal() {
    return (PIDUtil.checkWithinRange(getGoal(), getMeasurement(), ArmConstants.ANGLE_TOLERANCE_AUTON));
  }

  public double getAbsoluteEncoderPosition() {
    return armIO.getAbsoluteEncoderPosition();
  }

  public void setEncoderPosition(double angle) {
    armIO.setEncoderPosition(angle);
  }

  // TODO there's a duplicate of this in RealArm. Also, do we want this in
  // Robotçontainer instead?
  public double getSpeedFromArmHeight() {
    if (getEncoderPosition() <= 0.37) {
      speedFromArmHeight = Constants.ShooterConstants.SUBWOOFER_SPEED;
    } else if (getEncoderPosition() > 0.37 & getEncoderPosition() <= 0.76) {
      speedFromArmHeight = Constants.ShooterConstants.SPEAKER_SPEED;
    } else if (getEncoderPosition() > 0.76 & getEncoderPosition() <= 1) {
      speedFromArmHeight = Constants.ShooterConstants.FAR_AWAY_SPEED;
    } else if (getEncoderPosition() > 1) {
      speedFromArmHeight = Constants.ShooterConstants.AMP_SPEED;
    }
    return speedFromArmHeight;
  }

  // public void setArmAngle(DoubleSupplier desiredAngleSupplier) {
  // setGoal(desiredAngleSupplier.get());
  // }

  // 5.33E-03*x + 0.206
  // https://docs.google.com/spreadsheets/d/1TCEiHto6ypUku9VXPN79PGwONyrlhI2SbMsfn337yTw/edit#gid=0
  // inverse tan of function above to get angle

  public double getDesiredArmAnglePoseEstimation(Pose2d robotPose, Pose2d speakerPose) {
    double distToSpeaker;
    double desiredArmAngleRadians;
    distToSpeaker = PhotonUtils.getDistanceToPose(robotPose, speakerPose);
    if (distToSpeaker <= VISION_ARM_ALIGNMENT_STAY_SUBWOOF_POSITION_BOUNDARY_INCHES) {
      desiredArmAngleRadians = SPEAKER_SUBWOOFER_ANGLE;
    } else if (distToSpeaker <= VISION_ARM_ALIGNMENT_EIGHTY_PERCENT_SHOOTER_SPEED_MODEL_RANGE) {
      desiredArmAngleRadians = VISION_ARM_ALIGNMENT_EIGHTY_PERCENT_SHOOTER_SPEED_MODEL_SLOPE
          * (distToSpeaker)
          + VISION_ARM_ALIGNMENT_EIGHTY_PERCENT_SHOOTER_SPEED_MODEL_Y_INTERCEPT;
    } else {
      desiredArmAngleRadians = VISION_ARM_ALIGNMENT_HUNDRED_PERCENT_SHOOTER_SPEED_MODEL_SLOPE
          * (distToSpeaker)
          + VISION_ARM_ALIGNMENT_HUNDRED_PERCENT_SHOOTER_SPEED_MODEL_Y_INTERCEPT;
    }
    if (atGoal()) {
      desiredArmAngleRadians = getMeasurement(); // TODO: idk if this should be the auton value,,,but it seems like the
                                                 // right method to call
    }
    return desiredArmAngleRadians;
  }
}
