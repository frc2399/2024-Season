// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class automaticIntakeAndIndexer extends Command {
  private final Indexer indexer;
  private final Intake intake;
  public static boolean isIntooked = false;
  public boolean pieceIntooked = false;
  /** Creates a new automaticIntakeAndIndexer. */
  public automaticIntakeAndIndexer(Indexer indexer, Intake intake) {
    this.indexer = indexer;
    addRequirements(indexer);
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexer.getIsBeamBroken()) {
      indexer.setIsIntooked(true);
      isIntooked = true;
    }
    else 
    intake.setMotor(1);
    indexer.setMotor(1);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotor(0);
    indexer.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isIntooked;
  }
}
