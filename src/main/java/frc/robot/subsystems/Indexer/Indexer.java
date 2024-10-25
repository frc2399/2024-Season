// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private IndexerIO indexerIO;
  public boolean isIntooked = false;

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    indexerIO = io;
  }

  public Command setIsIntooked(boolean intooked) {
    return new InstantCommand(() -> isIntooked = intooked);
  }

  public boolean getIsIntooked() {
    return isIntooked;
  }

  public double getCurrent() {
    return indexerIO.getCurrent();
  }

  public void setMotor(double indexerSpeed) {
    indexerIO.setMotor(indexerSpeed);
  }

  public void setIsOverride() {
    indexerIO.setIsOverride();
  }

  // returns speed of the indexer
  public double getEncoderSpeed() {
    return indexerIO.getEncoderSpeed();
  }

  public void setCurrentLimit(int current) {
    indexerIO.setCurrentLimit(current);
  }

  public boolean getIsBeamBroken() {
    return indexerIO.getIsBeamBroken();
  }

  @Override
  public void periodic() {
    indexerIO.periodicUpdate();
    SmartDashboard.putBoolean("indexer/ isIntooked", isIntooked);
  }

  public Command runIndexer(double speed) {
    return this.run(() -> indexerIO.setMotor(speed));
  }
}
