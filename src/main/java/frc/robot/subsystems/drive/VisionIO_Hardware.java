package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Constants.VisionConstants;

public class VisionIO_Hardware implements VisionIO {
  public final PhotonCamera camera;
  private final PhotonPoseEstimator camEstimator;
  private final AprilTagFieldLayout KFIELDLAYOUT;

  public VisionIO_Hardware() {
    camera = new PhotonCamera("backup_camera"); // swap if swapping cameras
    camEstimator = new PhotonPoseEstimator(VisionConstants.KFIELDLAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.CAMTOROBOT);
    KFIELDLAYOUT = VisionConstants.KFIELDLAYOUT;
  }

  public Optional<EstimatedRobotPose> getVisionPose() {
    return camEstimator.update();
  }
}
