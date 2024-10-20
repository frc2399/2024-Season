package frc.robot.subsystems.drive;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class VisionIO_Hardware implements VisionIO {
  public final PhotonCamera camera;
  private final PhotonPoseEstimator camEstimator;

  public VisionIO_Hardware() {
    camera = new PhotonCamera("backup_camera"); // swap if swapping cameras
    camEstimator = new PhotonPoseEstimator(VisionConstants.KFIELDLAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.ROBOT_TO_CAM);
  }

  public Optional<EstimatedRobotPose> getVisionPose() {
    return camEstimator.update();
  }
}