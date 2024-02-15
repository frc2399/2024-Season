package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSim extends SubsystemBase implements VisionIO {

    // A vision system sim labelled as "main" in NetworkTables
    PhotonCamera camera = new PhotonCamera("simulated_camera");

    VisionSystemSim visionSim = new VisionSystemSim("front_camera");
    public static AprilTagFieldLayout aprilTagFieldLayout;
    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    double camDiagFOV = 100.0; // degrees
    double camPitch = CAMERA_PITCH_RADIANS; // degrees
    double camHeightOffGround = CAMERA_HEIGHT_METERS; // meters
    double minTargetArea = 0.1; // percentage (0 - 100)
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    PhotonCameraSim cameraSim;

    private static AprilTagFieldLayout kFieldLayout;
    private static PhotonPoseEstimator camPoseEstimator;
    private boolean updatePoseWithVisionReadings = true;

    /** Creates a new Vision. */
    public VisionSim() {
        visionSim.addAprilTags(aprilTagFieldLayout);

        // Get the built-in Field2d used by this VisionSystemSim
        visionSim.getDebugField();

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
                camResolutionWidth, camResolutionHeight, Rotation2d.fromDegrees(camDiagFOV));
        cameraProp.setCalibError(0.2, 0.05);
        cameraProp.setFPS(25);
        cameraProp.setAvgLatencyMs(30);
        cameraProp.setLatencyStdDevMs(4);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values
        // with visible
        // targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp, minTargetArea, maxLEDRange);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
        // pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z =
        // 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        camPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera, robotToCamera);

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);
        camPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.camToRobot);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
        SmartDashboard.putBoolean("Pose Updates Enabled?: ", updatePoseWithVisionReadings);
        camPoseEstimator.update();
        // System.out.println("working!!!");
        if (!updatePoseWithVisionReadings) {
            return;
        }
        // var result = camera.getLatestResult();
        // if (result.hasTargets()) {
        // var target = result.getBestTarget();
        // SmartDashboard.putNumber("result", target.getYaw());
        // }
    }

    // gets latest result
    public PhotonPipelineResult getCameraResult() {
        return camera.getLatestResult();
    }

    // estimates the robot pose
    public Optional<EstimatedRobotPose> getCameraEst() {
        var visionest = camPoseEstimator.update();
        System.out.println("vision est:");
        System.out.println(visionest);
        return visionest;
    }

    // creates functions that allow the changing of whether or not to use vision to
    // update the pose - NEED CONDITIONS AND/OR BUTTON
    public void enableUpdatePoseWithVisionReading() {
        updatePoseWithVisionReadings = true;
    }

    public void disableUpdatePoseWithVisionReading() {
        updatePoseWithVisionReadings = false;
    }

    // allows the camera instance into other commands
    public PhotonCamera getCamera() {
        return camera;
    }

}
