package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionReal;
    

public class GetToTarget extends Command {
    private DriveSubsystem drive;
    private VisionReal vision;
    private int targetID;
    private boolean seesTarget = false;
    private PhotonTrackedTarget desiredTarget;
    private Pose3d camEstimate;
    private Translation2d poseDiff;
    private double xDiff;
    private double yDiff;
    private double kProportional = 0.1;
    private ProfiledPIDController robotXLocController = new ProfiledPIDController(kProportional, 0, 0, 
        new TrapezoidProfile.Constraints(3.0,3.0));
    private ProfiledPIDController robotYLocController = new ProfiledPIDController(kProportional, 0, 0, 
        new TrapezoidProfile.Constraints(3.0,3.0));
    final double ANGULAR_P = 0.8; // TODO: tune
    final double ANGULAR_D = 0.0;
    private ProfiledPIDController robotRotationController = new ProfiledPIDController(
          ANGULAR_P, 0, ANGULAR_D, 
          new TrapezoidProfile.Constraints(
            Units.degreesToRadians(540.0), 
            Units.degreesToRadians(540.0)));
    private double yawDiff;
    private double desiredXOffset;
    private double desiredYOffset;

    public GetToTarget(DriveSubsystem drive, VisionReal vision, int targetID, double desiredXOffset, double desiredYOffset) {
        this.drive = drive;
        this.vision = vision;
        this.targetID = targetID;
        this.desiredXOffset = desiredXOffset;
        this.desiredYOffset = desiredYOffset;
        camEstimate = vision.robotPose;
        addRequirements(drive, vision);
    }

    @Override
    public void initialize() {
        for (PhotonTrackedTarget result : vision.getCameraResult().getTargets()) {
            if (result.getFiducialId() == targetID) {
                seesTarget = true;
                desiredTarget = result;
                drive.alignOrigins(camEstimate.toPose2d());
                poseDiff = new Translation2d(
                    camEstimate.getX() - VisionConstants.kFieldLayout.getTagPose(targetID).get().toPose2d().getX(),
                    camEstimate.getY() - VisionConstants.kFieldLayout.getTagPose(targetID).get().toPose2d().getY()
                    );
                if (xDiff < 0) {
                    xDiff = poseDiff.getX() + desiredXOffset;
                } else {
                    xDiff = poseDiff.getX() - desiredXOffset;
                }
                if (yDiff < 0) {
                    yDiff = poseDiff.getY() + desiredXOffset;
                } else {
                    yDiff = poseDiff.getY() - desiredYOffset;
                }
                yawDiff = desiredTarget.getYaw();
                break; //saves a tiny bit of processing power possibly
            }
        }
    if (!seesTarget) {
      end(true);
    }
    }

    public void execute() {
        drive.drive(
            robotXLocController.calculate(xDiff, 0),
            robotYLocController.calculate(yDiff, 0),
            robotRotationController.calculate(yawDiff, 0), 
            true);
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        if (robotXLocController.atGoal() && robotYLocController.atGoal() && robotRotationController.atGoal()) {
            return true;
        } else {
            return false;
        }
    }
}