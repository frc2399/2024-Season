package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public double keepPointedAtSpeaker(int speakerID) {
        SmartDashboard.putNumber("Speaker ID from Vision (should be 7) ", speakerID);
        return io.keepPointedAtSpeaker(speakerID);
    }
    public double keepArmAtAngle(int SpeakerID) {
        return io.keepArmAtAngle(SpeakerID);
    }

}