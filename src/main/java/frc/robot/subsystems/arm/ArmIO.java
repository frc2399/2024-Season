package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public interface ArmIO {
    public double getEncoderPosition();

    public double getEncoderSpeed();

    public void setSpeed(double speed);

    public void periodicUpdate();

    public double getArmCurrent();

    public double getAbsoluteEncoderPosition();

    public void setEncoderPosition(double angle);

    public double getDesiredArmAngle(Pose2d robotPose, Pose2d speakerPose);

}
