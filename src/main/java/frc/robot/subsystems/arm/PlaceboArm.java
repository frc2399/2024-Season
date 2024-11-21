package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public class PlaceboArm implements ArmIO {
    public double getEncoderPosition() {
        return 0;
    }

    public double getEncoderVelocity() {
        return 0;
    }

    public void setSpeed(double speed) {
    }

    public void periodicUpdate() {
    }

    public double getArmCurrent() {
        return 0;
    }

    public double getAbsoluteEncoderPosition() {
        return 0;
    }

    public void setEncoderPosition(double angle) {
    }

    public double getDesiredArmAngle(Pose2d robotPose, Pose2d speakerPose) {
        return 0;
    }
}
