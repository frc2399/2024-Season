// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionObsolete extends SubsystemBase {

  /** Creates a new Vision. */
  public VisionObsolete() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
    // SmartDashboard.putBoolean("Pose Updates Enabled?: ",
    // updatePoseWithVisionReadings);
  }

  // checks if there are targets in the current result
  public Boolean hasTargets() {
    return getCameraResult().hasTargets();
  }

  // gets the target determined to be the 'best'
  public PhotonTrackedTarget bestTarget() {
    return getCameraResult().getBestTarget();
  }

  // gets all visible targets in list ordered from 'best' to 'worst'
  public List<PhotonTrackedTarget> getTargets() {
    return getCameraResult().getTargets();
  }

  // creates functions that allow the changing of whether or not to use vision to
  // update the pose - find an actual use for this at some point
  public void enableUpdatePoseWithVisionReading() {
    updatePoseWithVisionReadings = true;
  }

  public void disableUpdatePoseWithVisionReading() {
    updatePoseWithVisionReadings = false;
  }

  // keeps the robot pointed at the speaker; uses PID and yaw
  public double keepPointedAtSpeaker() {
    boolean seesSpeaker = false;
    double yawDiff = 0.0;
    // gets yaw to centralized speaker target
    for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
      if (result.getFiducialId() == speakerID) {
        seesSpeaker = true;
        // yaw in radians bc p values get too small
        yawDiff = ((result.getYaw() * Math.PI) / 180);
        // Add green/red square for if robot aligned within 5 degrees to speaker tag
        if (yawDiff < Math.toRadians(5)) {
          isAligned = true;
          driveTrainIsAligned = true;
        } else {
          isAligned = false;
          driveTrainIsAligned = false;
        }
        break; // saves a tiny bit of processing power possibly
      }
    }
    if (!seesSpeaker) {
      driveTrainIsAligned = false;
    }
    return (keepPointedController.calculate(yawDiff, 0));
  }

  // retrns desired arm radians based on distance from aprilTag
  public double keepArmAtAngle(double curArmAngle) {
    final double EIGHTYSLOPE = VisionConstants.EIGHTYMODELSLOPE;
    final double EIGHTYINTERCEPT = VisionConstants.EIGHTYMODELINTERCEPT;
    final double HUNDREDSLOPE = VisionConstants.HUNDREDMODELSLOPE;
    final double HUNDREDINTERCEPT = VisionConstants.HUNDREDMODELINTERCEPT;
    final double BOUNDARY = VisionConstants.EIGHTYMODELRANGE;
    final double STAYDOWNBOUNDARY = VisionConstants.STAYDOWNBOUNDARY;
    double dist;
    Translation2d speakerDist;
    PhotonTrackedTarget speakerTarget;
    boolean seesSpeaker = false;
    double desiredRadians = 0.31;
    // this should help with the debugging :)
    for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
      if (result.getFiducialId() == speakerID) {
        seesSpeaker = true;
        speakerTarget = result;
        // gets the translation from the robot's current (x,y) to the (x,y) of the
        // speaker-center
        speakerDist = speakerTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
        // gets distance + calculates models (returning desired arm)
        dist = speakerDist.getNorm();
        // accounts for model measuring from front of frame and pose being to center of
        // robot
        dist -= Units.inchesToMeters(15.75);
        dist = Units.metersToInches(dist);
        if (dist <= STAYDOWNBOUNDARY) {
          desiredRadians = 0.31;
        } else if (dist <= BOUNDARY) {
          desiredRadians = ((EIGHTYSLOPE * (dist) + EIGHTYINTERCEPT));
        } else {
          desiredRadians = ((HUNDREDSLOPE * (dist) + HUNDREDINTERCEPT));
        }
      }
    }
    if (!seesSpeaker) {
      armIsAligned = false;
      desiredRadians = curArmAngle;
    }
    if (Math.abs(curArmAngle - desiredRadians) < VisionConstants.ARMALIGNTOLERANCE) {
      armIsAligned = true;
    } else {
      armIsAligned = false;
    }
    return desiredRadians;
  }
}