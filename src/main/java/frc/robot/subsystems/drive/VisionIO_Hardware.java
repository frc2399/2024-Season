package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.*;
//import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIO_Hardware implements VisionIO {
  // public final PhotonCamera camera;
  // private final PhotonPoseEstimator camEstimator;
  private final String limelightName;
  private static final double CAMERA_OFFSET_TO_ROBOT_X_METERS = Units.inchesToMeters(-11.94);
  private static final double CAMERA_OFFSET_TO_ROBOT_Y_METERS = Units.inchesToMeters(-7.54);
  private static final double CAMERA_OFFSET_TO_ROBOT_Z_METERS = Units.inchesToMeters(0);
  private static final double CAMERA_OFFSET_TO_ROBOT_ROLL_RADIANS = Units.degreesToRadians(24.62);
  private static final double CAMERA_OFFSET_TO_ROBOT_PITCH_RADIANS = 0;
  private static final double CAMERA_OFFSET_TO_ROBOT_YAW_RADIANS = 0;

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