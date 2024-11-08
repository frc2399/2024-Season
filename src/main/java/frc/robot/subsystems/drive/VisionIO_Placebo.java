package frc.robot.subsystems.drive;

import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIO_Placebo implements VisionIO {
  public PoseEstimate getVisionPose(double robotYaw) {
    return new PoseEstimate();
  }
}