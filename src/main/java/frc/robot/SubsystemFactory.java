package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.arm.SimArm;

public class SubsystemFactory {

    private final boolean isSim;

    ArmIO armIO;

    public SubsystemFactory(boolean isSim) {
        this.isSim = isSim;
    }

    protected Arm buildArm() {
        if (isSim) {
            armIO = new SimArm();
        } else {
            armIO = new RealArm();

        }
        return new Arm(armIO);
    }
}
