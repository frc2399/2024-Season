package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants; 
import frc.utils.MotorUtil;

public class RealIndexer implements IndexerIO {

    public static CANSparkMax indexerMotorController;
    public static RelativeEncoder indexerEncoder;
    public static SparkPIDController indexerController;
    private double slewRate = 0.2;
    public boolean isIntooked = false;
    public boolean isSensorOverriden = false;
    private static DigitalInput indexerSensorTop;
    private static DigitalInput indexerSensorBottom;

    public RealIndexer() {
        indexerMotorController = MotorUtil.createSparkMAX(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless,
                Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        indexerEncoder = indexerMotorController.getEncoder();
        indexerController = indexerMotorController.getPIDController();
        indexerController.setFeedbackDevice(indexerEncoder);
        indexerController.setFF(0.0001);
        indexerSensorTop = new DigitalInput(IndexerConstants.INDEXER_SENSOR_CHANNEL_TOP);
        indexerSensorBottom = new DigitalInput(IndexerConstants.INDEXER_SENSOR_CHANNEL_BOTTOM);
    }

    @Override
    public void setMotor(double indexerSpeed) {
        indexerMotorController.set(indexerSpeed);
    }

    public void setSpeed(double speedPercent) {
        indexerController.setReference(speedPercent * Constants.NEO550_MAX_SPEED_RPM, ControlType.kVelocity);
        SmartDashboard.putNumber("shooter/reference", speedPercent);
        SmartDashboard.putNumber("shooter/speed (RPM)", getEncoderSpeed() / Constants.NEO550_MAX_SPEED_RPM);
    }

    public double getCurrent() {
        return indexerMotorController.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
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
        SmartDashboard.putBoolean("isIntooked:", isIntooked);
        SmartDashboard.putBoolean("getIsBeamBroken", getIsBeamBroken());
        SmartDashboard.putNumber("indexer current", getCurrent());
    }

    @Override
    public void setIsIntooked(boolean intooked) {
        isIntooked = intooked;
    }

    @Override
    public boolean getIsBeamBroken() {
        if (isSensorOverriden) {
            return false;
        } else {
            return indexerSensorBottom.get();
        }
    }

    @Override
    public boolean getIsIntooked() {
        return isIntooked;
    }

    @Override
    public boolean isStalling() {
        return false;
    }
}
