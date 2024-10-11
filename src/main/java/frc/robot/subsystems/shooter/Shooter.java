// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;
  public static boolean isIntooked = false;

  /** Creates a new Intake. */
  public Shooter(ShooterIO io) {
    shooterIO = io;
  }

  public void setMotor(double shooterSpeed) {
    shooterIO.setMotor(shooterSpeed);
  }

  // rewritten from above
  public Command setMotors(double shooterSpeed) {
    return this.run(() -> shooterIO.setMotor(shooterSpeed));
  }

  // returns speed of the intake
  public double getEncoderSpeed() {
    return shooterIO.getEncoderSpeed();
  }

  public void setCurrentLimit(int current) {
    shooterIO.setCurrentLimit(current);
  }

  // rewritten from above
  public Command setCurrentLimit(int current) {
    return this.run(() -> shooterIO.setCurrent(current));
  }

  @Override
  public void periodic() {
    shooterIO.periodicUpdate();
  }
}
