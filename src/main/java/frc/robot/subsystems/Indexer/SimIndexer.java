package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.subsystems.indexer.SimIndexer;
import frc.utils.SimEncoder;

public class SimIndexer implements IndexerIO {
    public static SimEncoder indexerEncoderSim;
    private DCMotorSim indexerMotorSim;
    private PIDController pidController;
    public boolean isIntooked = false;

    public SimIndexer() {
        indexerEncoderSim = new SimEncoder("indexer");
        indexerMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
        SmartDashboard.putNumber("indexer current sim", 0);
        SmartDashboard.putNumber("indexer sim velocity", 0);
        pidController = new PIDController(1, 0,0);
    }
    
    public void setIsIntooked(boolean intooked) {
        isIntooked = intooked;
    }
    
    public void setMotor(double speed) {
        indexerMotorSim.setInput(speed);
    }

    public void setSpeed(double speed) {
        pidController.calculate(getEncoderSpeed(), speed);
    }

    public double getCurrent() {
        return indexerMotorSim.getCurrentDrawAmps();
    }

    public double getEncoderSpeed() {
        return indexerEncoderSim.getSpeed();
    }

    public double getEncoderPosition() {
        return indexerEncoderSim.getDistance();
    }

    public void setCurrentLimit(int current) {
        return;
    }

    public void periodicUpdate() {
        SmartDashboard.putNumber("indexer/current (A)", getCurrent());
    }

    public boolean getIsBeamBroken() {
        return false; 
    }

    @Override
    public void setIsSensorOverriden(boolean override) {
    }

    @Override
    public boolean getIsIntooked() {
        return false;
    }

    @Override
    public boolean getIsSensorOverriden() {
        return false;
    }
}
