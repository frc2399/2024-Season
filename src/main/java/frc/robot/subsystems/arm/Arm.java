// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.utils.PIDUtil;

public class Arm extends ProfiledPIDSubsystem {
  private ArmIO ArmIO;

  public static double speedFromArmHeight;

  // TODO tune max_vel and max_accel
  private static final double MAX_VEL = 4; // rad/s (NEO specs / gear ratio, converted into rad/s ~ 4.1, give it a
                                           // slightly lower one to make it acheivable)
  private static final double MAX_ACCEL = 6; // rad/s/s TODO: figure out where this number came from!
  private static final Constraints velAndAccelConstraints = new Constraints(MAX_VEL, MAX_ACCEL);
  private static double GRAVITY_COMPENSATION = 0.025;
  private static double FEED_FORWARD = 1 / MAX_VEL;
  private static double ARM_P = 3.0;
  private static double ARM_D = 0.01;
  private static final double ANGLE_TOLERANCE_AUTON = Units.degreesToRadians(2);

  public Arm(ArmIO io) {
    super(new ProfiledPIDController(ARM_P, 0, ARM_D, velAndAccelConstraints));
    ArmIO = io;
  }

  @Override
  public void periodic() {
    // Call periodic method in profile pid subsystem to prevent overriding
    super.periodic();
    ArmIO.periodicUpdate();
  }

  public double getEncoderPosition() {
    return ArmIO.getEncoderPosition();
  }

  public double getEncoderVelocity() {
    return ArmIO.getEncoderVelocity();
  }

  public Command setSpeed(double speed) {
    return this.run(() -> ArmIO.setSpeed(speed));
  }

  // TODO: add comments
  public Command setSpeedGravityCompensation(double speed) {
    return this.run(() -> setSpeed(speed + GRAVITY_COMPENSATION * Math.cos(getEncoderPosition())));
  }

  public double getArmCurrent() {
    return ArmIO.getArmCurrent();
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    SmartDashboard.putNumber("arm/desired position (deg)", Math.toDegrees(setpoint.position));
    SmartDashboard.putNumber("arm/desired velocity (deg per s)", Math.toDegrees(setpoint.velocity));

    double speed = FEED_FORWARD * setpoint.velocity;
    // accounts for gravity in speed
    speed += GRAVITY_COMPENSATION * Math.cos(getEncoderPosition());
    speed += output;
    setSpeed(speed);
  }

  @Override
  protected double getMeasurement() {
    return ArmIO.getEncoderPosition();
  }

  public double getGoal() {
    return m_controller.getGoal().position;
  }

  // Checks to see if arm is within range of the setpoints
  public boolean atGoal() {
    return (PIDUtil.checkWithinRange(getGoal(), getMeasurement(), ANGLE_TOLERANCE_AUTON));
  }

  public double getAbsoluteEncoderPosition() {
    return ArmIO.getAbsoluteEncoderPosition();
  }

  // why does this return a command? it's just a setter
  public Command setEncoderPosition(double angle) {
    return this.run(() -> ArmIO.setEncoderPosition(angle));
  }

  // TODO: move to command factory
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

  public Command makeSetPositionCommand(double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {
        }), new InstantCommand(() -> enable()), () -> isEnabled()),
        new RunCommand(() -> setGoal(target)));
  }

  public Command makeSetPositionCommandAuton(double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {
        }), new InstantCommand(() -> enable()), () -> isEnabled()),
        new InstantCommand(() -> setGoal(target)),
        new WaitCommand(0.5));
  }

  public Command makeSetSpeedGravityCompensationCommand(double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> disable()),
        new RunCommand(() -> setSpeedGravityCompensation(speed)));
  }

}
