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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionReal extends SubsystemBase implements VisionIO {
    private static AprilTagFieldLayout kFieldLayout;
    public static PhotonCamera camera;
    private static PhotonPoseEstimator CamEstimator;
    private boolean updatePoseWithVisionReadings = true;
    public Pose2d robotPose;
    // public Pose2d prevRobotPose;
    final double ANGULAR_P = 0.8; // TODO: tune
    final double ANGULAR_D = 0.0;
    ProfiledPIDController keepPointedController = new ProfiledPIDController(
      ANGULAR_P, 0, ANGULAR_D, 
      new TrapezoidProfile.Constraints(
        Units.degreesToRadians(540.0), 
        Units.degreesToRadians(540.0)));

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
    //System.out.println(CamEstimator.update());
    //if (!updatePoseWithVisionReadings) {
      // return;}
    if (pose.isPresent()) {
    var result = camera.getLatestResult();
     if (result.hasTargets()) {
      //System.out.println(pose);
      PhotonTrackedTarget target = bestTarget();
      //the robot pose is estimating field to robot using photon utils
      // if (robotPose != null) { // TODO: better way w/ less processing power - maybe smth in the init idk
      // prevRobotPose = robotPose;
      // }
      robotPose = PhotonUtils.estimateFieldToRobot(
        new Transform2d(
          new Translation2d(pose.get().estimatedPose.getX(), pose.get().estimatedPose.getY()), 
          pose.get().estimatedPose.getRotation().toRotation2d()),
         kFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d(), VisionConstants.camToRobot2d);
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

    public double keepPointedAtSpeaker(int SpeakerID) {
      int speakerID = SpeakerID;
      boolean seesSpeaker = false;
      double yawDiff = 0.0;
      for (PhotonTrackedTarget result : getCameraResult().getTargets()) {
        if (result.getFiducialId() == speakerID) {
          seesSpeaker = true;
          //yaw in radians bc p values get too small
          yawDiff = ((result.getYaw()*Math.PI)/180);
          SmartDashboard.putBoolean("Sees speaker (only true when in keep pointed mode): ", true);
          break; //saves a tiny bit of processing power possibly
        }
      }
      if (!seesSpeaker) {
        SmartDashboard.putBoolean("Sees speaker (only true when in keep pointed mode): ", false);
      }
      return (keepPointedController.calculate(yawDiff, 0));
    }

    public double keepArmAtAngle(int SpeakerID) {    
      final double eightySlope = VisionConstants.eightyModelSlope;
      final double eightyIntercept = VisionConstants.eightyModelIntercept;
      final double hundredSlope = VisionConstants.hundredModelSlope;
      final double hundredIntercept = VisionConstants.hundredModelIntercept;
      final double boundary = VisionConstants.eightyModelRange;
      final int desiredSpeakerTag = SpeakerID;
      double dist;
      Translation2d speakerDist = new Translation2d(
        robotPose.getX() - VisionConstants.kFieldLayout.getTagPose(desiredSpeakerTag).get().toPose2d().getX(),
        robotPose.getY() - VisionConstants.kFieldLayout.getTagPose(desiredSpeakerTag).get().toPose2d().getY()
      );
      dist = speakerDist.getNorm();
      if (dist <= boundary) {
        return (Math.atan(eightySlope * Units.metersToInches(dist) + eightyIntercept));
      } else {
        return (Math.atan(hundredSlope * Units.metersToInches(dist) + hundredIntercept));
      }
    }
}