// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionReal extends SubsystemBase implements VisionIO {
    private static AprilTagFieldLayout kFieldLayout;
    public static PhotonCamera camera;
    private static PhotonPoseEstimator CamEstimator;
    private boolean updatePoseWithVisionReadings = true;

    /** Creates a new Vision. */
    public VisionReal() {
        camera = new PhotonCamera("Arducam_OV2311_USB_Camera");
        CamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.camToRobot);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
        SmartDashboard.putBoolean("Pose Updates Enabled?: ", updatePoseWithVisionReadings);
        var pose = CamEstimator.update();
        // System.out.println("working!!!");
        if (!updatePoseWithVisionReadings) {
            return;
        }
        // var result = camera.getLatestResult();
        // if (result.hasTargets()) {
        // var target = result.getBestTarget();
        // SmartDashboard.putNumber("result", target.getYaw());
        // }
    }

    // gets latest result
    public PhotonPipelineResult getCameraResult() {
        return camera.getLatestResult();
    }

    // estimates the robot pose
    public Optional<EstimatedRobotPose> getCameraEst() {
        System.out.println(CamEstimator);
        var visionest = CamEstimator.update();
        System.out.println("vision est:");
        System.out.println(visionest);
        return visionest;
    }

    // creates functions that allow the changing of whether or not to use vision to
    // update the pose - NEED CONDITIONS AND/OR BUTTON
    public void enableUpdatePoseWithVisionReading() {
        updatePoseWithVisionReadings = true;
    }

    public void disableUpdatePoseWithVisionReading() {
        updatePoseWithVisionReadings = false;
    }

    // allows the camera instance into other commands
    public PhotonCamera getCamera() {
        return camera;
    }
}
