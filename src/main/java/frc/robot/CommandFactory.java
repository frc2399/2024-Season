package frc.robot;

import frc.robot.subsystems.arm.Arm;

public class CommandFactory {
    private final Arm arm;

    public CommandFactory(Arm arm) {
        this.arm = arm;
    }

    public double getSpeedFromArmHeight() {
        double speedFromArmHeight = 0;
        if (arm.getEncoderPosition() <= 0.37) {
            speedFromArmHeight = Constants.ShooterConstants.SUBWOOFER_SPEED;
        } else if (arm.getEncoderPosition() > 0.37 & arm.getEncoderPosition() <= 0.76) {
            speedFromArmHeight = Constants.ShooterConstants.SPEAKER_SPEED;
        } else if (arm.getEncoderPosition() > 0.76 & arm.getEncoderPosition() <= 1) {
            speedFromArmHeight = Constants.ShooterConstants.FAR_AWAY_SPEED;
        } else if (arm.getEncoderPosition() > 1) {
            speedFromArmHeight = Constants.ShooterConstants.AMP_SPEED;
        }
        return speedFromArmHeight;
    }
}
