package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.SimEncoder;

public class SimShooter implements ShooterIO {

    public static SimEncoder shooterEncoderSim;
    private DCMotorSim shooterMotorSim;

    public SimShooter() {
        shooterEncoderSim = new SimEncoder("shooter");
        shooterMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    }

    public void setMotor(double speed) {
        shooterMotorSim.setInput(speed);
    }

    public double getCurrent() {
        return SmartDashboard.getNumber("shooter/current sim (A)", -100);
    }

    public double getEncoderSpeed() {
        return SmartDashboard.getNumber("shooter/sim velocity", -100);
    }

    public void setCurrentLimit(int current) {
        SmartDashboard.getNumber("shooter/current limit sim (A)", current);
    }

    public void periodicUpdate() {
        SmartDashboard.putNumber("shooter/current (A)", getCurrent());
    }
}
