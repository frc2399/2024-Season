package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utils.SimEncoder;

public class SwerveModuleIO_Sim implements SwerveModuleIO {

   // Simualaion Motors
   private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.ModuleConstants.kDrivingMotorReduction,
         0.025);
   private DCMotorSim turnMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.ModuleConstants.kTurningMotorReduction,
         0.025);

   private SimEncoder m_turningEncoder;
   private SimEncoder m_drivingEncoder;
   // Random initial position to simulate arbitrary starting positions
   private final Rotation2d turnAbsoluteInitialPosition = new Rotation2d(Math.random() * 2 * Math.PI); // Random initial
                                                                                                       // position

   // Variables to track various module information
   private double driveVolts = 0.0;
   private double turnVolts = 0.0;
   private String name;

   // Sim-Tuned PID Controllers
   private PIDController m_turningPIDController = new PIDController(15, 0.0, 0.0);
   private PIDController m_drivingPIDController = new PIDController(5, 0.0, 0.0);
   private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0,
         Constants.ModuleConstants.kDrivingFF * 12);

   private SwerveModuleState setpoint = new SwerveModuleState();

   private double driveMotorOutput = 0.0;
   private double turnMotorOutput = 0.0;

   public SwerveModuleIO_Sim(String name) {
      m_drivingEncoder = new SimEncoder(name + " drive encoder");
      m_turningEncoder = new SimEncoder(name + " turn encoder");
      this.name = name;
   }

   public void updateInputs(SwerveModuleIOInputs inputs) {

   };

   public void setDriveEncoderPosition(double position) {
      m_drivingEncoder.setDistance(position);
   };

   public double getDriveEncoderPosition() {
      return m_drivingEncoder.getDistance();
   };

   public double getDriveEncoderSpeedMPS() {
      return m_drivingEncoder.getSpeed();
   };

   public double getTurnEncoderPosition() {
      return m_turningEncoder.getDistance();
   };

   public void setDesiredDriveSpeedMPS(double speed) {
      driveMotorOutput = m_drivingPIDController.calculate(getDriveEncoderSpeedMPS(), speed);
      SmartDashboard.putNumber(name + " desired drive speed", speed);
      SmartDashboard.putNumber(name + " actual drive speed", getDriveEncoderSpeedMPS());
      SmartDashboard.putNumber(name + "motor speed", driveMotorOutput);

      // Apply PID output
      driveMotor.setInputVoltage(driveMotorOutput);
      driveMotor.getAngularVelocityRPM();
      driveMotor.update(0.02);
      SmartDashboard.putNumber(name + "driveMotor.getAngularVelocityRPM", driveMotor.getAngularVelocityRPM());

   };

   public void setDesiredTurnAngle(double angle) {
      turnMotorOutput = m_turningPIDController.calculate(getTurnEncoderPosition(), angle);

      // Apply PID output
      turnMotor.setInputVoltage(turnMotorOutput);
   };

   public double getDriveBusVoltage() {
      return 0;
   };

   public double getDriveOutput() {
      return driveMotorOutput;
   }

   public double getTurnBusVoltage() {
      return 0;
   }

   public double getTurnOutput() {
      return turnMotorOutput;
   }
}
