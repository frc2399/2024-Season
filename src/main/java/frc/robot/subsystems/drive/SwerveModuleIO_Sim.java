package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SwerveModuleIO_Sim implements SwerveModuleIO{

    // Simualaion Motors
    private DCMotorSim driveMotor =
      new DCMotorSim(DCMotor.getNEO(1), Constants.ModuleConstants.kDrivingMotorReduction, 0.025);
    private DCMotorSim turnMotor =
      new DCMotorSim(DCMotor.getNeo550(1), Constants.ModuleConstants.kTurningMotorReduction, 0.025);

    // Random initial position to simulate arbitrary starting positions
    private final Rotation2d turnAbsoluteInitialPosition =
      new Rotation2d(Math.random() * 2 * Math.PI); // Random initial position

    // Variables to track various module information
    private double driveVolts = 0.0;
    private double turnVolts = 0.0;

    // Sim-Tuned PID Controllers
    private PIDController turnController = new PIDController(15, 0.0, 0.0);
    private PIDController driveController = new PIDController(5, 0.0, 0.0);
    private SimpleMotorFeedforward driveFeedforward =
        new SimpleMotorFeedforward(0.0, Constants.ModuleConstants.kDrivingFF * 12);

    private SwerveModuleState setpoint = new SwerveModuleState();
    
    public void updateInputs(SwerveModuleIOInputs inputs){

    };
    
    public void setDriveEncoderPosition(double position){

    };
        
     public double getDriveEncoderPosition(){
        return 0; 
     };
      
     public double getDriveEncoderSpeedMPS(){
        return 0; 
     };
        
     public double getTurnEncoderPosition(){
        return 0; 
     };
 
     public void setDesiredDriveSpeedMPS(double speed){

     };
     public void setDesiredTurnAngle(double angle){

     };
     public double getDriveVolts(){
        return 0;
     };

     public double getDriveOutput(){
        return 0;
     }
     
     public double getTurnVolts(){
        return 0;
     }
}
