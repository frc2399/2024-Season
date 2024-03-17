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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionReal extends SubsystemBase implements VisionIO {
    private static AprilTagFieldLayout KFIELDLAYOUT;
    public static PhotonCamera camera;
    private static PhotonPoseEstimator CamEstimator;
    private boolean updatePoseWithVisionReadings = true;
    public Pose3d robotPose;
    private boolean driveTrainIsAligned = false;
    private boolean armIsAligned = false;
    

    //apriltags
    public  int facingSourceLeftID;
    public  int facingSourceRightID;
    public  int speakerID;
    public  int speakerOffsetID;
    public  int stageBackID;
    public  int facingAwayFromSpeakerStageLeftID;
    public   int facingAwayFromSpeakerStageRightID;
    public  int ampID;
    public boolean isAligned= false;

    //PID for the speaker-aiming method
    final double ANGULAR_P = 0.8; // TODO: tune
    final double ANGULAR_D = 0.0;
    PIDController keepPointedController = new PIDController(
      ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new Vision. */
  public VisionReal() {
    camera = new PhotonCamera("backup_camera"); //swap if swapping cameras
    CamEstimator = new PhotonPoseEstimator(VisionConstants.KFIELDLAYOUT, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.CAMTOROBOT);
    KFIELDLAYOUT = VisionConstants.KFIELDLAYOUT; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
    //SmartDashboard.putBoolean("Pose Updates Enabled?: ", updatePoseWithVisionReadings);
    if (!updatePoseWithVisionReadings) {
      return;
    }
    Optional<EstimatedRobotPose> pose = getCameraEst();
      
    //makes sure that there is a new pose and that there are targets before getting a robot pose 
    if (pose.isPresent()) {
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        PhotonTrackedTarget target = bestTarget();
        target.getBestCameraToTarget();
        //the robot pose is estimating field to robot using photon utils
        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),
          KFIELDLAYOUT.getTagPose(target.getFiducialId()).get(), 
          VisionConstants.CAMTOROBOT);
     }
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

    //checks if there are targets in the current result
    public Boolean hasTargets() {
      return getCameraResult().hasTargets(); 
    }
    
    //gets the target determined to be the 'best'   
    public PhotonTrackedTarget bestTarget() {
      return getCameraResult().getBestTarget();
    }
    
    //gets all visible targets in list ordered from 'best' to 'worst'
    public List<PhotonTrackedTarget> getTargets() {
      return getCameraResult().getTargets();
    }

  //creates functions that allow the changing of whether or not to use vision to update the pose - find an actual use for this at some point 
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

    //keeps the robot pointed at the speaker; uses PID and yaw
    public double keepPointedAtSpeaker() {
      boolean seesSpeaker = false;
      double yawDiff = 0.0;
      //gets yaw to centralized speaker target
      for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
        if (result.getFiducialId() == speakerID) {
          seesSpeaker = true;
          //yaw in radians bc p values get too small
          yawDiff = ((result.getYaw()*Math.PI)/180);
          // Add green/red square for if robot aligned within 5 degrees to speaker tag
          if (yawDiff < Math.toRadians(5)) {
            isAligned = true;
            driveTrainIsAligned = true;
          }
          else {
            isAligned = false;
            driveTrainIsAligned = false;
          }
          break; //saves a tiny bit of processing power possibly
        }
      }
      if (!seesSpeaker) {
        driveTrainIsAligned = false;
      }
      return (keepPointedController.calculate(yawDiff, 0));
    }

  //retrns desired arm radians based on distance from aprilTag
  public double keepArmAtAngle(double curArmAngle) {    
      final double EIGHTYSLOPE = VisionConstants.EIGHTYMODELSLOPE;
      final double EIGHTYINTERCEPT = VisionConstants.EIGHTYMODELINTERCEPT;
      final double HUNDREDSLOPE = VisionConstants.HUNDREDMODELSLOPE;
      final double HUNDREDINTERCEPT = VisionConstants.HUNDREDMODELINTERCEPT;
      final double BOUNDARY = VisionConstants.EIGHTYMODELRANGE;
      double dist;
      Translation2d speakerDist;
      PhotonTrackedTarget speakerTarget;
      boolean seesSpeaker = false;
      double desiredRadians = 0.37;
      //this should help with the debugging :)
      for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
        if (result.getFiducialId() == speakerID) {
          seesSpeaker = true;
          speakerTarget = result;
            //gets the translation from the robot's current (x,y) to the (x,y) of the speaker-center
            speakerDist = speakerTarget.getBestCameraToTarget().getTranslation().toTranslation2d();            
            //gets distance + calculates models (returning desired arm)
            dist = speakerDist.getNorm();
            //accounts for model measuring from front of frame and pose being to center of robot
            dist -= Units.inchesToMeters(15.75);
            if (dist <= BOUNDARY) {
              desiredRadians = (Math.atan(EIGHTYSLOPE * Units.metersToInches(dist) + EIGHTYINTERCEPT));
            } else {
              desiredRadians = (Math.atan(HUNDREDSLOPE * Units.metersToInches(dist) + HUNDREDINTERCEPT));
            }
        }
      }
      if (!seesSpeaker) {
        armIsAligned = false;
      }
      if (Math.abs(curArmAngle - desiredRadians) < VisionConstants.ARMALIGNTOLERANCE) {
        armIsAligned = true;
      } else {
        armIsAligned = false;
      }
      return desiredRadians;
    }

    public boolean isDriveTrainAligned() {
      return driveTrainIsAligned;
    }

    public boolean isArmAligned() {
      return armIsAligned;
    }

    //assigns aprilTags based on alliance
    public void assignAprilTags(Optional<Alliance> ally) {
      if (ally.get() == Alliance.Red) {
          facingSourceLeftID = 10;
          facingSourceRightID = 9;
          speakerID = 4;
          speakerOffsetID = 3;
          stageBackID = 13;
          facingAwayFromSpeakerStageLeftID = 11;
          facingAwayFromSpeakerStageRightID = 12;
          ampID = 5;
        } else {
          facingSourceLeftID = 1;
          facingSourceRightID = 2;
          speakerID = 7;
          speakerOffsetID = 8;
          stageBackID = 14;
          facingAwayFromSpeakerStageLeftID = 15;
          facingAwayFromSpeakerStageRightID = 16;
          ampID = 6;
      }
    }

    public void makeDriveTrainAlignedFalse() {
      driveTrainIsAligned = false;
    }

    public void makeArmAlignedFalse() {
      armIsAligned = false;
    }
  }