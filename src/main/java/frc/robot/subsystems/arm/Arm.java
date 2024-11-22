// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.utils.PIDUtil;

public class Arm extends ProfiledPIDSubsystem {
  private ArmIO ArmIO;

  public static double speedFromArmHeight;

  // TODO tune max_vel and max_accel
  private static final double MAX_VEL = 2;// original: 4, changed lower for testing purposes // rad/s (NEO specs / gear
                                          // ratio, converted into rad/s ~ 4.1, give it a
                                          // slightly lower one to make it acheivable)
  private static final double MAX_ACCEL = 4;// original: 6, changed lower for testing purposes // rad/s/s pretty sure
                                            // this number is just an estimate
  private static final Constraints velAndAccelConstraints = new Constraints(MAX_VEL, MAX_ACCEL);
  private static double GRAVITY_COMPENSATION = 0.025;
  private static double FEED_FORWARD = 0;
  private static double ARM_P = 1;
  private static double ARM_D = 0;
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
    SmartDashboard.putBoolean("arm/arm enabled", isEnabled());
  }

  public double getEncoderPosition() {
    return ArmIO.getEncoderPosition();
  }

  public double getEncoderVelocity() {
    return ArmIO.getEncoderVelocity();
  }

  // gravity compensation is a factor multiplied by vertical component of the
  // encoder position to
  // add scaled speed to the arm the in order to overcome the acceleration due to
  // gravity
  public Command setSpeedGravityCompensation(double speed) {
    return this.run(() -> ArmIO.setSpeed(speed + GRAVITY_COMPENSATION * Math.cos(getEncoderPosition())));
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
    ArmIO.setSpeed(speed);
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

  public Command setEncoderPosition(double angle) {
    return this.run(() -> ArmIO.setEncoderPosition(angle));
  }

  public Command makeSetPositionCommand(double target) {
    return Commands.sequence(
        Commands.print("Sequential Cmd Group started!"),
        Commands.either(Commands.none(), Commands.runOnce(() -> enable(), this), () -> isEnabled()),
        Commands.print("Either Cmd run!"),
        Commands.runOnce(() -> setGoal(target)),
        Commands.print("Set Goal run!"));
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

  public double getDesiredArmAngle(Pose2d robotPose, Pose2d speakerPose) {
    return ArmIO.getDesiredArmAngle(robotPose, speakerPose);
  }

}
