package frc.robot.subsystems.drive;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
  public Optional<EstimatedRobotPose> getVisionPose();
}