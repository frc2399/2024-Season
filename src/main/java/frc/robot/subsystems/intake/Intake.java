// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;
  public static boolean isIntooked = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {   
    intakeIO = io;        
  }

  public void setMotor(double intakeSpeed) {
    intakeIO.setMotor(intakeSpeed);
  }

  //returns speed of the intake
  public double getLeftEncoderSpeed() {
    return intakeIO.getLeftEncoderSpeed();
  }

  public double getRightEncoderSpeed() {
    return intakeIO.getRightEncoderSpeed();
  }

  public void setLeftCurrentLimit(int current) {
    intakeIO.setLeftCurrentLimit(current);
  }

  public void setRightCurrentLimit(int current) {
    intakeIO.setRightCurrentLimit(current);
  }

  @Override
  public void periodic() {
    intakeIO.periodicUpdate();
  }
}
