// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public interface VisionIO {
    public void periodic();
    public PhotonPipelineResult getCameraResult();
    public Optional<EstimatedRobotPose> getCameraEst();
    public void enableUpdatePoseWithVisionReading ();
    public void disableUpdatePoseWithVisionReading ();
    public PhotonCamera getCamera () ;
}
