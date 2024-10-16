// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;
  public static boolean isIntooked = false;

  /** Creates a new Intake. */
  public Shooter(ShooterIO io) {
    shooterIO = io;
  }

  public Command setMotor(double shooterSpeed) {
    return this.run(() -> shooterIO.setMotor(shooterSpeed));
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
    shooterIO.periodicUpdate();
  }

  public Command setShootSpeed(double speed) {
    return this.run(() -> shooterIO.setMotor(speed));
  }

  public boolean isUpToSpeed(double speed) {
    return shooterIO.getEncoderSpeed() >= (speed); // Does not have a shooter max rps
  }
}
