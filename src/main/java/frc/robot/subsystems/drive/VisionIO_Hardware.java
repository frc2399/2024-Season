package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.*;
//import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIO_Hardware implements VisionIO {
  // public final PhotonCamera camera;
  // private final PhotonPoseEstimator camEstimator;
  private final String limelightName;
  private static final double CAMERA_OFFSET_TO_ROBOT_X_METERS = Units.inchesToMeters(-1);
  private static final double CAMERA_OFFSET_TO_ROBOT_Y_METERS = Units.inchesToMeters(-10);
  private static final double CAMERA_OFFSET_TO_ROBOT_Z_METERS = Units.inchesToMeters(8.75);
  private static final double CAMERA_OFFSET_TO_ROBOT_ROLL_RADIANS = 0;
  private static final double CAMERA_OFFSET_TO_ROBOT_PITCH_RADIANS = Units.degreesToRadians(43.2);
  private static final double CAMERA_OFFSET_TO_ROBOT_YAW_RADIANS = Units.degreesToRadians(180);

  public VisionIO_Hardware() {
    limelightName = "limelight";
    LimelightHelpers.setCameraPose_RobotSpace(limelightName, CAMERA_OFFSET_TO_ROBOT_X_METERS,
        CAMERA_OFFSET_TO_ROBOT_Y_METERS, CAMERA_OFFSET_TO_ROBOT_Z_METERS, CAMERA_OFFSET_TO_ROBOT_ROLL_RADIANS,
        CAMERA_OFFSET_TO_ROBOT_PITCH_RADIANS, CAMERA_OFFSET_TO_ROBOT_YAW_RADIANS);
  }

  public PoseEstimate getVisionPose(double robotYaw) {
    LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
  }
}