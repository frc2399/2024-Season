package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.SimEncoder;

public class IntakeSim implements IntakeIO {

    public static SimEncoder leftIntakeEncoderSim;
    public static SimEncoder rightIntakeEncoderSim;
    private DCMotorSim leftIntakeMotorSim;
    private DCMotorSim rightIntakeMotorSim;

    public IntakeSim() {
        leftIntakeEncoderSim = new SimEncoder("left intake");
        rightIntakeEncoderSim = new SimEncoder("right intake");
        leftIntakeMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
        rightIntakeMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
        SmartDashboard.putNumber("intake current sim", 0);
        SmartDashboard.putNumber("intake sim velocity", 0);

    }

    public void setMotor(double speed) {
        leftIntakeMotorSim.setInput(speed);
        //rightIntakeMotorSim.setInput(speed);
    }
   
    public double getLeftCurrent() {
        return leftIntakeMotorSim.getCurrentDrawAmps();
    }

    public double getRightCurrent() {
        return rightIntakeMotorSim.getCurrentDrawAmps();
    }
    
    public double getLeftVelocity() {
        return leftIntakeEncoderSim.getSpeed();
    }

    public double getRightVelocity() {
        return rightIntakeEncoderSim.getSpeed();
    }
   
    public double getLeftEncoderPosition() {
        return leftIntakeEncoderSim.getDistance();
    }

    public double getRightEncoderPosition() {
        return rightIntakeEncoderSim.getDistance();
    }
   
    public void setLeftCurrentLimit(int current) {
        return;
    }

     public void setRightCurrentLimit(int current) {
        return;
    }

    public void periodicUpdate() {
        SmartDashboard.putNumber("intake/current (A)", getLeftCurrent());
        SmartDashboard.putNumber("intake/current (A)", getRightCurrent());
    }
    
    @Override
    public void setIntakePID(double percentOutput) {
        
    }
}
