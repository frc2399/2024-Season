package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants; 

public class IndexerReal implements IndexerIO {

    public static CANSparkMax indexerMotorController;
    public static RelativeEncoder indexerEncoder;
    public static SparkPIDController indexerController;
    public boolean isIntooked = false;
    public boolean isSensorOverriden = false;
    private double FEEDFORWARD = 0.000925;
    private double PVALUE = 0.0000925;
    private static DigitalInput indexerSensorBottom;
    boolean isIdleBreak;

    public IndexerReal() {
        indexerMotorController = new CANSparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
        indexerMotorController.restoreFactoryDefaults();
        indexerMotorController.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
        indexerMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // initialize motor encoder
        indexerEncoder = indexerMotorController.getEncoder();
        indexerEncoder.setVelocityConversionFactor(2*Math.PI/60.0); //convert to rps
        indexerController = indexerMotorController.getPIDController();
        indexerController.setFeedbackDevice(indexerEncoder);
        indexerController.setFF(FEEDFORWARD);
        indexerController.setP(PVALUE);

       // indexerSensorTop = new DigitalInput(IndexerConstants.INDEXER_SENSOR_CHANNEL_TOP);
        indexerSensorBottom = new DigitalInput(IndexerConstants.INDEXER_SENSOR_CHANNEL_BOTTOM);
        indexerMotorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 32767);
        indexerMotorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 32767);
        indexerMotorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 32767);
        indexerMotorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 32767);
        indexerMotorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 32767);
    }


    @Override
    public void setMotor(double indexerVelocity) {
        indexerMotorController.set(indexerVelocity);

        //SmartDashboard.putNumber("indexer/set speed", indexerVelocity);
    }


    public double getCurrent() {
        return indexerMotorController.getOutputCurrent();
    }

    @Override
    public double getVelocity() {
        return indexerEncoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return indexerEncoder.getPosition();
    }

    @Override
    public void setCurrentLimit(int current) {
        indexerMotorController.setSmartCurrentLimit(current);
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putBoolean("indexer/isIntooked:", isIntooked);
        SmartDashboard.putBoolean("indexer/getIsBeamBroken", getIsBeamBroken());
        SmartDashboard.putNumber("indexer/encoder value", indexerEncoder.getVelocity());
    }

    @Override
    public boolean getIsBeamBroken() {
        if (isSensorOverriden) {
            return false;
        } else {
            return !indexerSensorBottom.get();
        }
    }

    @Override
    public void setIsOverride() {
        isSensorOverriden = !isSensorOverriden;
    }

    @Override
    public void setIndexerPID(double indexerVelocity)
    {
        indexerController.setReference(indexerVelocity, ControlType.kVelocity);
        SmartDashboard.putNumber("indexer/set speed", indexerVelocity);
    }
}
