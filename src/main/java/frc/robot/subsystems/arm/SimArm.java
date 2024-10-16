package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.SimEncoder;

public class SimArm implements ArmIO {

    private SimEncoder armEncoderSim;
    private SingleJointedArmSim armSim;
    private double armPower;
    public static double speedFromArmHeight;

    private static final double INITIAL_OFFSET = 0.274;
    private static final double MAX_ARM_ANGLE = Math.PI / 4 * 3; // in radians
    private static final double MIN_ARM_ANGLE = 0;
    private static final double ARM_MASS = 2.72155; // in kg
    public static final double ARM_LENGTH = 0.65; // in meters

    public SimArm() {
        armEncoderSim = new SimEncoder("Elevator");
        armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1), // 1 NEO motor on the climber
                75,
                SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS),
                ARM_LENGTH,
                MIN_ARM_ANGLE,
                MAX_ARM_ANGLE,
                true, INITIAL_OFFSET);
    }

    @Override
    public double getEncoderPosition() {
        return armEncoderSim.getDistance();
    }

    @Override
    public double getEncoderVelocity() {
        return armEncoderSim.getSpeed();
    }

    @Override
    public void setSpeed(double speed) {
        armPower = speed;
    }

    @Override
    public void periodicUpdate() {
        // sets input for elevator motor in simulation
        armSim.setInput(armPower * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.02);
        // Finally, we set our simulated encoder's readings
        armEncoderSim.setDistance(armSim.getAngleRads());
        // sets our simulated encoder speeds
        armEncoderSim.setSpeed(armSim.getVelocityRadPerSec());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }

    @Override
    public double getArmCurrent() {
        return armSim.getCurrentDrawAmps();
    }

    @Override
    public double getAbsoluteEncoderPosition() {
        return 0;
    }

    @Override
    public void setEncoderPosition(double angle) {
    }
}
