package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveModuleIO;

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

}