// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class automaticIntakeAndIndexer extends Command {
  private final Indexer indexer;
  private final Intake intake;
  private final Arm arm;
  public static boolean isIntooked;
  public boolean pieceIntooked = false;
  /** Creates a new automaticIntakeAndIndexer. */
  public automaticIntakeAndIndexer(Indexer indexer, Intake intake, Arm arm) {
    this.indexer = indexer;
    addRequirements(indexer);
    this.intake = intake;
    addRequirements(intake);
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if((arm.getEncoderPosition() < 0.5) && indexer.isIntooked == false) {
          isIntooked = false;
      }
      else {
        isIntooked = true;
      }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexer.getIsBeamBroken()) {
      indexer.setIsIntooked(true);
      isIntooked = true;
    }
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
