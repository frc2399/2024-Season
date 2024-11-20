// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;
  public static boolean isIntooked = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {   
    intakeIO = io;        
  }

  // percent output goes from -1 - 1
  public void setMotor(double percentOutput) {
    intakeIO.setMotor(percentOutput);
  }

  //returns speed of the intake
  public double getLeftVelocity() {
    return intakeIO.getLeftVelocity();
  }

  public double getRightVelocity() {
    return intakeIO.getRightVelocity();
  }

  public void setLeftCurrentLimit(int current) {
    intakeIO.setLeftCurrentLimit(current);
  }

  public void setRightCurrentLimit(int current) {
    intakeIO.setRightCurrentLimit(current);
  }

  public Command setIntakePID(double percentOutput) {
    return this.run(() -> intakeIO.setIntakePID(percentOutput));
  }

  @Override
  public void periodic() {
    intakeIO.periodicUpdate();
    SmartDashboard.putNumber("intake/left velocity", getLeftVelocity());
    SmartDashboard.putNumber("intake/right velocity", getRightVelocity());

  }
}
