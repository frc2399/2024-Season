package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.utils.SimEncoder;

public class SimArm implements ArmIO{

    private SimEncoder armEncoderSim;
    private SingleJointedArmSim armSim;
    private double armPower;
    public static double speedFromArmHeight;

    public SimArm() {
        armEncoderSim = new SimEncoder("Elevator");
        armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1), // 1 NEO motor on the climber
                75,
                SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH, ArmConstants.ARM_MASS),
                ArmConstants.ARM_LENGTH,
                ArmConstants.MIN_ARM_ANGLE,
                ArmConstants.MAX_ARM_ANGLE,
                true, ArmConstants.INITIAL_OFFSET);
    }

    @Override
    public double getEncoderPosition() {
        return armEncoderSim.getDistance();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoderSim.getSpeed();
    }

    @Override
    public void setSpeed(double speed) {
        armPower = speed;        
    }

    @Override
    public void periodicUpdate(){
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
    public double getSpeedFromArmHeight() {
        if (getEncoderPosition() < 0.4) {
            speedFromArmHeight = 0.7;
          } else if (getEncoderPosition() < 0.8 & getEncoderPosition() > 0.4) {
            speedFromArmHeight = 0.8;
          } else if (getEncoderPosition() < 1 & getEncoderPosition() > 0.8) {
            speedFromArmHeight = 1;
          } else if (getEncoderPosition() > 1) {
              speedFromArmHeight = 0.3;
          }
          return speedFromArmHeight;
    }

    @Override
    public double getAbsoluteEncoderPosition() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setEncoderPosition(double angle) {
        // TODO Auto-generated method stub
    } 
}
