package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.ModuleConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModuleIO_Real implements SwerveModuleIO {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final SparkPIDController m_turningPIDController;

    private double chassisAngularOffset;

    private String name;

    public SwerveModuleIO_Real(int drivingCANId, int turningCANId, double chassisAngularOffset,
      String name){

      this.name = name;
      
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);


    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // m_drivingSparkMax.enableVoltageCompensation(12);
    // m_turningSparkMax.enableVoltageCompensation(12);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
    //also testing lol
    m_drivingSparkMax.setInverted(ModuleConstants.kDrivingEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    this.chassisAngularOffset = chassisAngularOffset;
    m_drivingEncoder.setPosition(0);
    }
    


    public void updateInputs(SwerveModuleIOInputs inputs){
        inputs.driveBusVoltage = getDriveBusVoltage();
        inputs.driveAppliedVolts = getDriveBusVoltage() * getDriveOutput();
        inputs.drivePositionMeters = getDriveEncoderPosition();
        inputs.driveVelocityMPS = getDriveEncoderSpeedMPS();
        inputs.turnBusVoltage = getTurnBusVoltage();
        inputs.turnAppliedVolts = getTurnBusVoltage() * getTurnOutput();
        inputs.turnPositionRad = getTurnEncoderPosition();
        
    }
    
    //TODO: check units on these methods 
    public void setDriveEncoderPosition(double position){
        m_drivingEncoder.setPosition(position);
    };
        
     public double getDriveEncoderPosition(){
        return m_drivingEncoder.getPosition();
     };
      
     public void setDesiredDriveSpeedMPS(double speed){
        m_drivingPIDController.setReference(speed, ControlType.kVelocity);
     };

     public double getDriveEncoderSpeedMPS(){
        return m_drivingEncoder.getVelocity();
     };
        
     public double getTurnEncoderPosition(){
        return m_turningEncoder.getPosition();
     };
     

     public void setDesiredTurnAngle(double angle){
        m_turningPIDController.setReference(angle, ControlType.kPosition);
     };


     public double getDriveBusVoltage(){
        return m_drivingSparkMax.getBusVoltage();
     }

     public double getDriveOutput(){
        return m_drivingSparkMax.getAppliedOutput();
     }

     public double getTurnBusVoltage(){
        return m_turningSparkMax.getBusVoltage();
     }

     public double getTurnOutput(){
         return m_turningSparkMax.getAppliedOutput();
     }

     public String getName(){
      return name;
   }

   public double getChassisAngularOffset(){
      return chassisAngularOffset;
   }

}


