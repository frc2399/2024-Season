// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.PIDUtil;

public class Arm extends ProfiledPIDSubsystem {
  /** Creates a new Arm. */
  private ArmIO armIO;
  private double targetAngle = -Math.PI / 2;
  // private static final double feedForward = 0.133;
  // private static final double feedForward = 0.14285;
  // private static final double kpPos = 0.8;
  public static double speedFromArmHeight;

  // Trapezoidal profile constants and variables
  private static final double max_vel = 3.9; // rad/s (NEO specs / gear ratio, converted into rad/s ~ 4.1, give it a slightly lower one to make it acheivable)
  private static final double max_accel = 2.7; // rad/s/s
  private static final Constraints constraints = new Constraints(max_vel, max_accel);
  private static double gravityCompensation = 0.025;
  private static double feedForward = 1;
  private static double kpPos = 3.0;
  private static double kd = 0.01;

  public Arm(ArmIO io) {
    super(new ProfiledPIDController(kpPos, 0, kd, constraints));
    armIO = io;
    SmartDashboard.putNumber("kG", 0);
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kd", 0);
    SmartDashboard.putNumber("arm ff", 0);
  }

  @Override
  public void periodic() {
    // Call periodic method in profile pid subsystem to prevent overriding
    super.periodic();
    armIO.periodicUpdate();

    // gravityCompensation = SmartDashboard.getNumber("kG", 0);
    kpPos = SmartDashboard.getNumber("kP", 0);
    kd = SmartDashboard.getNumber("kd", 0);
    feedForward = SmartDashboard.getNumber("arm ff", 0);
    SmartDashboard.putNumber("arm/goal position", getGoal());
    SmartDashboard.putNumber("arm/velocity", getEncoderSpeed());
    SmartDashboard.putNumber("arm/postion", getEncoderPosition());

  }

  public double getEncoderPosition() {
    return armIO.getEncoderPosition();
  }

  public double getEncoderSpeed() {
    return armIO.getEncoderSpeed();
  }

  public void setSpeed(double speed) {
    //speed = Math.max(Math.min(speed, 0.5), -0.5);
    armIO.setSpeed(speed);
    SmartDashboard.putNumber("arm/speed", speed);
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public void setTargetAngle(double angle) {
    DataLogManager.log("Set target to " + angle);
    targetAngle = angle;
  }

  public void setSpeedGravityCompensation(double speed) {
    // calls set speed function in the file that does armIO.setSpeed after capping
    // speed
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
    // calls set speed function in the file that does armIO.setSpeed after capping
    // speed
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

  public void setPosition(double position) {
    armIO.setPosition(position);
  }

  public void setkG(double kG) {
    gravityCompensation = kG;
  }

  public double getAbsoluteEncoderPosition() {
    // TODO Auto-generated method stub
    return armIO.getAbsoluteEncoderPosition();
  }

public void setEncoderPosition(double angle) {
    // TODO Auto-generated method stub
    armIO.setEncoderPosition(angle);
}

  public double getSpeedFromArmHeight() {
    if (getEncoderPosition() <= 0.37) {
      speedFromArmHeight = Constants.ShooterConstants.subWooferShotSpeed;
    } else if (getEncoderPosition() > 0.37 & getEncoderPosition() <= 0.8) {
      speedFromArmHeight = Constants.ShooterConstants.speakerSpeed;
    } else if (getEncoderPosition() > 0.8 & getEncoderPosition() <= 1) {
      speedFromArmHeight = Constants.ShooterConstants.farAwayShotSpeed;
    } else if (getEncoderPosition() > 1) {
        speedFromArmHeight = Constants.ShooterConstants.ampSpeed;
    }
    return speedFromArmHeight;
    }

}
