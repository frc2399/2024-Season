// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private ShooterIO shooterIO;
  public static boolean isIntooked = false;

  /** Creates a new Intake. */
  public ShooterSubsystem(ShooterIO io) {
    shooterIO = io;
  }

  public void setMotor(double shooterSpeed) {
    shooterIO.setMotor(shooterSpeed);
  }

  public double getEncoderSpeed() {
    return shooterIO.getEncoderSpeed();
  }

  public Command shooterDefaultStopCommand() {
    return new RunCommand(
        () -> shooterIO.setMotor(0),
        this);
  }

  @Override
  public void periodic() {
    shooterIO.periodicUpdate();
  }
}
