package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    VisionIO io;

    public Vision(VisionIO io) {

        this.io = io;
        // this.name = io.getName();

    }
    
    public PhotonPipelineResult getCameraResult() {
        return io.getCameraResult();
    }
    public Optional<EstimatedRobotPose> getCameraEst() {
        return io.getCameraEst();
    }
    public void enableUpdatePoseWithVisionReading () {
        io.enableUpdatePoseWithVisionReading();
    }
    public void disableUpdatePoseWithVisionReading () {
        io.disableUpdatePoseWithVisionReading();
    }
    public PhotonCamera getCamera () {
        return io.getCamera();
    } 

    public double keepPointedAtSpeaker() {
        return io.keepPointedAtSpeaker();
    }
    public double keepArmAtAngle(double curArmAngle) {
        return io.keepArmAtAngle(curArmAngle);
    }

    public boolean isDriveTrainAligned() {
        return io.isDriveTrainAligned();
    }

    public boolean isArmAligned() {
        return io.isArmAligned();
    }

    public void assignAprilTags(Optional<Alliance> ally) {
      io.assignAprilTags(ally);
    }

    public void makeDriveTrainAlignedFalse() {
        io.makeDriveTrainAlignedFalse();
    }

    public void makeArmAlignedFalse() {
        io.makeArmAlignedFalse();
    }

}