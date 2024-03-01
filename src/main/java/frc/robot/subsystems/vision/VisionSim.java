package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;


public class VisionSim extends SubsystemBase implements VisionIO {

    // A vision system sim labelled as "main" in NetworkTables
    PhotonCamera camera = new PhotonCamera("simulated_camera");
    final double ANGULAR_P = 0.8; // TODO: tune
    final double ANGULAR_D = 0.0;
    PIDController keepPointedController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    VisionSystemSim visionSim = new VisionSystemSim("front_camera");
    public static AprilTagFieldLayout aprilTagFieldLayout;
    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    final double camHeightOffGround = Units.inchesToMeters(24);
    // final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double camPitch = Units.degreesToRadians(0);
    public  int facingSourceLeftID;
    public  int facingSourceRightID;
    public  int speakerID;
    public  int speakerOffsetID;
    public  int stageBackID;
    public  int facingAwayFromSpeakerStageLeftID;
    public  int facingAwayFromSpeakerStageRightID;
    public  int ampID;

    double camDiagFOV = 70.0; // degrees
    // double camPitch = CAMERA_PITCH_RADIANS; // degrees
    // double camHeightOffGround = CAMERA_HEIGHT_METERS; // meters
    double minTargetArea = 0.1; // percentage (0 - 100)
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 1280; // pixels
    int camResolutionHeight = 720; // pixels
    PhotonCameraSim cameraSim;

    private static PhotonPoseEstimator camPoseEstimator;
    private boolean updatePoseWithVisionReadings = true;
    private static AprilTagFieldLayout kFieldLayout;

    public Pose2d robotPose;

    private DriveSubsystem drive;

    /** Creates a new Vision. */
    public VisionSim(DriveSubsystem drive) {
        this.drive = drive ;
        
        visionSim.addAprilTags(VisionConstants.kFieldLayout);
        kFieldLayout = VisionConstants.kFieldLayout;
        // Get the built-in Field2d used by this VisionSystemSim
        visionSim.getDebugField();

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
                camResolutionWidth, camResolutionHeight, Rotation2d.fromDegrees(camDiagFOV));
        cameraProp.setCalibError(0.2, 0.05);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(24);
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

        // camPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //         camera, robotToCamera);

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);
        camPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.camToRobot);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("visiontesting/speakerID", speakerID);
        // This method will be called once per scheduler run
        // SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
        SmartDashboard.putBoolean("Pose Updates Enabled?: ", updatePoseWithVisionReadings);
        Optional<EstimatedRobotPose> pose = getCameraEst();
        if (pose.isPresent()) {
    var result = camera.getLatestResult();
     if (result.hasTargets()) {
      //System.out.println(pose);
      PhotonTrackedTarget target = bestTarget();
      //the robot pose is estimating field to robot using photon utils
      // if (robotPose != null) { // TODO: better way w/ less processing power - maybe smth in the init idk
      // prevRobotPose = robotPose;
      // }
      robotPose = PhotonUtils.estimateFieldToRobot(
        new Transform2d(
          new Translation2d(pose.get().estimatedPose.getX(), pose.get().estimatedPose.getY()), 
          pose.get().estimatedPose.getRotation().toRotation2d()),
         kFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d(), VisionConstants.camToRobot2d);
     }
    }
        visionSim.update(drive.getPose());

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
        // System.out.println("vision est:");
        // System.out.println(visionest);
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
// TODO: sim implementation
    public Boolean hasTargets() {
        return getCameraResult().hasTargets(); 
    }

    public PhotonTrackedTarget bestTarget() {
        return getCameraResult().getBestTarget();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return getCameraResult().getTargets();
    }

    public double keepPointedAtSpeaker() {
       return 0.0;
    }

    public double keepArmAtAngle() {    
      final double eightySlope = VisionConstants.eightyModelSlope;
      final double eightyIntercept = VisionConstants.eightyModelIntercept;
      final double hundredSlope = VisionConstants.hundredModelSlope;
      final double hundredIntercept = VisionConstants.hundredModelIntercept;
      final double boundary = VisionConstants.eightyModelRange;
      double dist;
      // Added if statement to avoid sim crash from button bindings setup
      if (robotPose == null) {
        return eightyIntercept;
      }
      else {
        Translation2d speakerDist = new Translation2d(
            robotPose.getX() - VisionConstants.kFieldLayout.getTagPose(7).get().toPose2d().getX(),
            robotPose.getY() - VisionConstants.kFieldLayout.getTagPose(7).get().toPose2d().getY()
        );
        dist = speakerDist.getNorm();
        System.out.println(Units.metersToInches(dist));
        System.out.println(Math.atan(eightySlope * Units.metersToInches(dist) + eightyIntercept));
        if (dist <= boundary) {
            return (Math.atan(eightySlope * Units.metersToInches(dist) + eightyIntercept));
        } else {
            return (Math.atan(hundredSlope * Units.metersToInches(dist) + hundredIntercept));
        }
    }
    }

    @Override
    public void assignAprilTags(Optional<Alliance> ally) {
        
            if (ally.get() == Alliance.Red) {
                facingSourceLeftID = 10;
                facingSourceRightID = 9;
                speakerID = 4;
                speakerOffsetID = 3;
                stageBackID = 13;
                facingAwayFromSpeakerStageLeftID = 11;
                facingAwayFromSpeakerStageRightID = 12;
                ampID = 5;
              } else {
                facingSourceLeftID = 1;
                facingSourceRightID = 2;
                speakerID = 7;
                speakerOffsetID = 8;
                stageBackID = 14;
                facingAwayFromSpeakerStageLeftID = 15;
                facingAwayFromSpeakerStageRightID = 16;
                ampID = 6;
            }
          SmartDashboard.putNumber("robotcontainer/speaker id", speakerID);
      
          
    }

}
