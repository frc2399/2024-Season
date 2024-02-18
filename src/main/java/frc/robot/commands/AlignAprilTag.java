// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.gyro.GyroIOPigeon2;

public class AlignAprilTag extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private Boolean posTurn;
  private final Pose2d currentPose;
  double angleInDegrees;
  Rotation2d targetAngle;
  /** Creates a new AlignAprilTag. */
  public AlignAprilTag(DriveSubsystem m_driveSubsystem, Vision m_visionSubsystem, Pose2d poseEstimate) {
    //makes the global variables the same as the ones intook from the call
    driveSubsystem = m_driveSubsystem;
    currentPose = poseEstimate;
    vision = m_visionSubsystem;
    addRequirements(m_driveSubsystem, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d cameraEst2d;
    //gets camera-estimated pose
    var cameraEstimate = vision.getCameraEst();
    if (cameraEstimate.isPresent()) {
      //if there is a camera estimate, converts it to a 2dpose
      cameraEst2d = cameraEstimate.get().estimatedPose.toPose2d();
      //takes care of conflicting origins (odo vs PhotonLib)
      driveSubsystem.alignOrigins(cameraEst2d);
      //these next three lines get the angle of the tag in degrees
      Translation2d tagPose = new Translation2d(-0.04,5.55);
      targetAngle = new Rotation2d(tagPose.getX() - cameraEst2d.getX(), tagPose.getY() - cameraEst2d.getY());
      angleInDegrees = targetAngle.getDegrees();
      //adjusts if it gives a negative angle so comparison can happen
      if (angleInDegrees < 0) {
        angleInDegrees += 360;
      }
      double robotCurPose = driveSubsystem.getPose().getRotation().getDegrees();
      //takes care of gyro facing the other way + not wrapping
      robotCurPose = ((robotCurPose + 180)%360);
      if (robotCurPose<angleInDegrees) {
        posTurn = true;
      } else {
        posTurn = false;
      }
    } else {
      end(true);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: how fast can we get to an angle accurately (get help from subsystems people); start by feeding it faster rotate speeds
    if (posTurn == true) {
      driveSubsystem.drive(0,0,0.1,true);
    } else {
      driveSubsystem.drive(0,0,-0.1,true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Entered end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //abs value of our pose - desired pose < 5: ends
    double robotCurPose = driveSubsystem.getPose().getRotation().getDegrees();
    if (Math.abs(((robotCurPose+180)%360) - angleInDegrees) < 5) {  
      return true;
    } else {
    return false;
    }
  }
}