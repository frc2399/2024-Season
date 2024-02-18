// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
private static AprilTagFieldLayout kFieldLayout;
public static PhotonCamera camera;
private static PhotonPoseEstimator CamEstimator;
private boolean updatePoseWithVisionReadings = true;
public Pose2d robotPose;
public Pose2d prevRobotPose;

  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("Arducam_OV2311_USB_Camera");    
    CamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.camToRobot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
    //SmartDashboard.putBoolean("Pose Updates Enabled?: ", updatePoseWithVisionReadings);
    Optional<EstimatedRobotPose> pose = CamEstimator.update();
    //if (!updatePoseWithVisionReadings) {
      // return;}
    var result = camera.getLatestResult();
     if (result.hasTargets()) {
      PhotonTrackedTarget target = bestTarget();
      //the robot pose is estimating field to robot using photon utils
      if (robotPose != null) {
        prevRobotPose = robotPose;
      }
      robotPose = PhotonUtils.estimateFieldToRobot(
        new Transform2d(
          new Translation2d(pose.get().estimatedPose.getX(), pose.get().estimatedPose.getY()), 
          pose.get().estimatedPose.getRotation().toRotation2d()),
         kFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d(), VisionConstants.camToRobot2d);
     }
  }

  //gets latest result
   public PhotonPipelineResult getCameraResult() {
    return camera.getLatestResult();
  }

  //estimates the robot pose
    public Optional<EstimatedRobotPose> getCameraEst() {
      var visionest = CamEstimator.update();
      return visionest;
    }

    public Boolean hasTargets() {
      return getCameraResult().hasTargets(); 
    }
    
    public PhotonTrackedTarget bestTarget() {
      return getCameraResult().getBestTarget();
    }
    
    public List<PhotonTrackedTarget> getTargets() {
      return getCameraResult().getTargets();
    }

  //creates functions that allow the changing of whether or not to use vision to update the pose - NEED CONDITIONS AND/OR BUTTON 
    public void enableUpdatePoseWithVisionReading () {
      updatePoseWithVisionReadings = true;
    }
    public void disableUpdatePoseWithVisionReading () {
      updatePoseWithVisionReadings = false;
    }

    //allows the camera instance into other commands
    public PhotonCamera getCamera() {
      return camera;
    }

    public Double keepPointedAtSpeaker(int desiredSpeakerID) {
      int speakerID = desiredSpeakerID;
      Translation2d poseChange = new Translation2d(
        robotPose.getX()-prevRobotPose.getX(),robotPose.getY()-prevRobotPose.getY());
      double xDiff = poseChange.getX();
      double yDiff = poseChange.getY();
      double speakerY = VisionConstants.kFieldLayout.getTagPose(speakerID).get().getY();
      if ((Math.abs(xDiff) == Math.abs(yDiff)) || (robotPose.getY() == speakerY)) {
        return 0.0;
      }
        //might not want this variable
        //check if the *3 matters - x value appears to be more influential than y but want to check
      Double changesSquared = (Math.pow(poseChange.getX(),2)*3 + Math.pow(poseChange.getY(),2)); //add scalar perhaps
      Double directionToTurn = 0.0; // TODO: implement actual algorithm here!!!
      return directionToTurn;
      //don't turn IF: these should be taken care of
        //1. delta x = delta y
        //2. delta x = - delta y
        //3. delta x = 0
      //strafing on *either* side
        //going right turn left
        //going left turn right
      //on right side: (either y-change)
        //going left: turn right
        //going right:turn left
      //on left side (either y-change)
        //going right: turn left
        //going left: turn right
    }

}