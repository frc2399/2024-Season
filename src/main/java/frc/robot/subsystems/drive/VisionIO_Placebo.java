package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class VisionIO_Placebo implements VisionIO {
  Optional<EstimatedRobotPose> placeboPoseEstimate;

  public Optional<EstimatedRobotPose> getVisionPose() {
    return placeboPoseEstimate;
  }
}
