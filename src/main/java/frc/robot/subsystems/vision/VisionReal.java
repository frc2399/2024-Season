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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionReal extends SubsystemBase implements VisionIO {
    private static AprilTagFieldLayout kFieldLayout;
    public static PhotonCamera camera;
    private static PhotonPoseEstimator CamEstimator;
    private boolean updatePoseWithVisionReadings = true;
    public Pose2d robotPose;

    //apriltags
    public  int facingSourceLeftID;
    public  int facingSourceRightID;
    public  int speakerID;
    public  int speakerOffsetID;
    public  int stageBackID;
    public  int facingAwayFromSpeakerStageLeftID;
    public   int facingAwayFromSpeakerStageRightID;
    public  int ampID;

    //PID for the speaker-aiming method
    final double ANGULAR_P = 0.8; // TODO: tune
    final double ANGULAR_D = 0.0;
    PIDController keepPointedController = new PIDController(
      ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new Vision. */
  public VisionReal() {
    camera = new PhotonCamera("Arducam_OV2311_USB_Camera"); //swap if swapping cameras
    CamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.camToRobot);
    kFieldLayout = VisionConstants.kFieldLayout; 
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
        //the robot pose is estimating field to robot using photon utils
        robotPose = PhotonUtils.estimateFieldToRobot(
          new Transform2d(
            new Translation2d(pose.get().estimatedPose.getX(), pose.get().estimatedPose.getY()), 
            pose.get().estimatedPose.getRotation().toRotation2d()),
          kFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d(), VisionConstants.camToRobot2d);
        SmartDashboard.putNumber("robot pose", robotPose.getX());
        SmartDashboard.putNumber("robot pose y", robotPose.getY());
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
      //System.out.println(visionest);
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
      SmartDashboard.putNumber("visiontesting/speakerID", speakerID);
      SmartDashboard.putNumber("speaker ID from VisionReal (should be 7) ", speakerID);
      boolean seesSpeaker = false;
      double yawDiff = 0.0;
      //gets yaw to centralized speaker target
      for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
        if (result.getFiducialId() == speakerID) {
          seesSpeaker = true;
          //yaw in radians bc p values get too small
          yawDiff = ((result.getYaw()*Math.PI)/180);
          SmartDashboard.putBoolean("Sees speaker (only true when in keep pointed mode): ", true);
          SmartDashboard.putNumber("YawDiff", yawDiff);
          break; //saves a tiny bit of processing power possibly
        }
      }
      if (!seesSpeaker) {
        SmartDashboard.putBoolean("Sees speaker (only true when in keep pointed mode): ", false);
      }
      System.out.println(keepPointedController.calculate(yawDiff, 0));
      return (keepPointedController.calculate(yawDiff, 0));
    }

    public double keepArmAtAngle() {    
      final double eightySlope = VisionConstants.eightyModelSlope;
      final double eightyIntercept = VisionConstants.eightyModelIntercept;
      final double hundredSlope = VisionConstants.hundredModelSlope;
      final double hundredIntercept = VisionConstants.hundredModelIntercept;
      final double boundary = VisionConstants.eightyModelRange;
      double dist;
      boolean seesSpeaker = false;
      boolean targetPoseDNE = kFieldLayout.getTagPose(speakerID).isEmpty();
      boolean sevenPoseDNE = kFieldLayout.getTagPose(7).isEmpty();
      double desiredRadians = 0.37;
      //this should help with the debugging :)
      for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
        if (result.getFiducialId() == speakerID) {// FIXME: any alliance
          seesSpeaker = true;
          if (sevenPoseDNE) {
            SmartDashboard.putString("Oh no", "we cannot find the ID 7's pose :(");
          }
          else if (targetPoseDNE) {
            SmartDashboard.putString("Oh no","we cannot find the speakerID's pose");
          }
          else {
            SmartDashboard.putString("Oh no", "It appears there is another bug bc it can find both of the IDs");
            //gets the translation from the robot's current (x,y) to the (x,y) of the speaker-center
            Translation2d speakerDist = new Translation2d(
              robotPose.getX() - kFieldLayout.getTagPose(speakerID).get().getX(),
              robotPose.getY() - kFieldLayout.getTagPose(speakerID).get().getY()
            );
            
            //gets distance + calculates models (returning desired arm)
            dist = speakerDist.getNorm();
            //accounts for model measuring from front of frame and pose being to center of robot
            dist -= Units.inchesToMeters(15.75);
            SmartDashboard.putNumber("distance", dist);
            SmartDashboard.putNumber("radians", Math.atan(eightySlope * Units.metersToInches(dist) + eightyIntercept));
            if (dist <= boundary) {
              desiredRadians = (Math.atan(eightySlope * Units.metersToInches(dist) + eightyIntercept));
            } else {
              desiredRadians = (Math.atan(hundredSlope * Units.metersToInches(dist) + hundredIntercept));
            }
          }
        }
      }
      if (!seesSpeaker) {
        SmartDashboard.putBoolean("Sees speaker (only true when in keep pointed mode): ", false);
      }      
      return desiredRadians;
    }

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
    SmartDashboard.putNumber("robotcontainer/speaker id", speakerID);

    }
  }