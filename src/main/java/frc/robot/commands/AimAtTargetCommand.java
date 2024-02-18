// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionIO;

public class AimAtTargetCommand extends Command {

    final double ANGULAR_P = 0.05;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private DriveSubsystem drive;
    private VisionIO vision;

    /** Creates a new AimAtTargetCommand. */
    public AimAtTargetCommand(DriveSubsystem drive, VisionIO vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double yawTotal = 0;
        int numTargets = 0;

        for (PhotonTrackedTarget result : vision.getCameraResult().getTargets()) {
            yawTotal += result.getYaw();
            numTargets += 1;
            // SmartDashboard.putNumber("Vision/yaw", result.getYaw());
        }

        if (numTargets > 0) {
            double rotationSpeed = -turnController.calculate(yawTotal / numTargets, 0);
            drive.drive(0.0, 0.0, rotationSpeed, true);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}