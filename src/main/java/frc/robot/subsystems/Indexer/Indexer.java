// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private IndexerIO indexerIO;
  public static boolean isIntooked = false;

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    
    indexerIO = io;
        
  }

  public void setMotor(double indexerSpeed) {
    indexerIO.setMotor(indexerSpeed);
  }

  public void setSpeed(double speed) {
    indexerIO.setSpeed(speed);
  }

  //returns speed of the indexer
  public double getEncoderSpeed() {
    return indexerIO.getEncoderSpeed();
  }

  public void setCurrentLimit(int current) {
    indexerIO.setCurrentLimit(current);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexerIO.periodicUpdate();
  }
}
