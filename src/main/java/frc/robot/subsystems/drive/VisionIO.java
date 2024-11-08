package frc.robot.subsystems.drive;

import frc.utils.LimelightHelpers.PoseEstimate;

public interface VisionIO {
  public PoseEstimate getVisionPose(double robotYaw);
}