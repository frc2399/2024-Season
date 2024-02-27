// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putNumber("intake/motor input (%)", shooterSpeed);
  }

  public double getCurrent() {
    return shooterIO.getCurrent();
  }

  // returns speed of the intake
  public double getEncoderSpeed() {
    return shooterIO.getEncoderSpeed();
  }

  public void setCurrentLimit(int current) {
    shooterIO.setCurrentLimit(current);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake/current (A)", getCurrent());
    shooterIO.periodicUpdate();
  }

public void setSpeed(double speed) {
    shooterIO.setSpeed(speed);
}
}
