// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//import frc.robot.subsystems.*;

public class AlignAprilTag extends Command {
  //global drive subsystem
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private final Pose2d currentPose;
  double angleInDegrees;
  /** Creates a new AlignAprilTag. */
  public AlignAprilTag(DriveSubsystem m_driveSubsystem, Vision m_visionSubsystem, Pose2d poseEstimate) {
    //makes the global variable the same as the one intook from the call
    driveSubsystem = m_driveSubsystem;
    currentPose = poseEstimate;
    vision = m_visionSubsystem;
    addRequirements(m_driveSubsystem, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing :)");
    Pose2d cameraEst2d;
    //reset odo to global (0,0)
      var cameraEstimate = vision.getCameraEst();
      System.out.println(cameraEstimate);
      if (cameraEstimate.isPresent()) {
      //our pose
        cameraEst2d = cameraEstimate.get().estimatedPose.toPose2d();
        driveSubsystem.alignOrigins(cameraEst2d);
        // Translation2d trial = VisionConstants.kFieldLayout.getTagPose(7).toPose2d().getTranslation();
        Translation2d tagPose = new Translation2d(-0.04,5.55);
        Rotation2d targetAngle = new Rotation2d(tagPose.getX() - cameraEst2d.getX(), tagPose.getY() - cameraEst2d.getY());
        if(currentPose.getRotation().getDegrees() < 0){
          targetAngle = targetAngle.minus(targetAngle.times(2));
        }
        angleInDegrees = targetAngle.getDegrees() - 180; 
    } else {
      System.out.println("camera estimate dne :(");
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //turn to said angle
    driveSubsystem.drive(0,0,0.1,true,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Aligned!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //abs value of our pose - desired pose < 10: ends
    double robotCurPose = driveSubsystem.getPose().getRotation().getDegrees();
    if (Math.abs((robotCurPose%360) - angleInDegrees) < 5) {
      return true;
    } else {
    System.out.println(robotCurPose - angleInDegrees);
    System.out.println("is finished entered");
    return false;
    }
  }
}
