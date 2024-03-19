// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public interface VisionIO {
    public void periodic();
    public PhotonPipelineResult getCameraResult();
    public Optional<EstimatedRobotPose> getCameraEst();
    public Boolean hasTargets();
    public PhotonTrackedTarget bestTarget();
    public List<PhotonTrackedTarget> getTargets();
    public void enableUpdatePoseWithVisionReading ();
    public void disableUpdatePoseWithVisionReading ();
    public PhotonCamera getCamera();
    public double keepPointedAtSpeaker();
    public double keepArmAtAngle(double curArmAngle);
    public void assignAprilTags(Optional<Alliance> ally);
    public boolean isDriveTrainAligned();
    public boolean isArmAligned();
    public void makeDriveTrainAlignedFalse();
    public void makeArmAlignedFalse();
}
