package frc.robot.subsystems.drive;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class VisionIO_Placebo implements VisionIO {

  public Optional<EstimatedRobotPose> getVisionPose() {
    Optional<EstimatedRobotPose> placeboPoseEstimate = Optional.empty();
    return placeboPoseEstimate;
  }
}