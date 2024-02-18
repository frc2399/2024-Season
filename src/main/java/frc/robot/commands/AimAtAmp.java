package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionReal;
    

public class AimAtAmp extends Command {
    private DriveSubsystem drive;
    private VisionReal vision;
    private int ampID = RobotContainer.aprilTagAssignment.ampID;
    private boolean seesAmp = false;
    private PhotonTrackedTarget ampTarget;
    private Pose2d camEstimate;
    private Translation2d poseDiff;
    private double xDiff;
    private double yDiff;

    public AimAtAmp(DriveSubsystem drive, VisionReal vision) {
        this.drive = drive;
        this.vision = vision;
        camEstimate = vision.robotPose;
        addRequirements(drive, vision);
    }

    @Override
    public void initialize() {
        for (PhotonTrackedTarget result : vision.getCameraResult().getTargets()) {
            if (result.getFiducialId() == ampID) {
                seesAmp = true;
                ampTarget = result;
                drive.alignOrigins(camEstimate);
                poseDiff = new Translation2d(
                    camEstimate.getX() - VisionConstants.kFieldLayout.getTagPose(ampID).get().toPose2d().getX(),
                    camEstimate.getY() - VisionConstants.kFieldLayout.getTagPose(ampID).get().toPose2d().getY()
                    );
                xDiff = Units.metersToInches(poseDiff.getX());
                yDiff = Units.metersToInches(poseDiff.getY()) + VisionConstants.yOffsetToRobot + 2;
                break; //saves a tiny bit of processing power possibly
            }
        }
    if (!seesAmp) {
      end(true);
    }
    }

    public void execute() {
        //get swerve odo to take over from here
        //drive commands based on xDiff and yDiff
        //maybe a PID
        //make it stop when PID is over
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }
}
