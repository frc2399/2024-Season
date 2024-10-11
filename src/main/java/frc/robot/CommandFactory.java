package frc.robot;

import frc.robot.subsystems.arm.Arm;

public class CommandFactory {
    private final Arm arm;

    public CommandFactory(Arm arm) {
        this.arm = arm;
    }
}
