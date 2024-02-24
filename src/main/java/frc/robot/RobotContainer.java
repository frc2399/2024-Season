// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot.RobotType;
import frc.robot.commands.automaticClimberCommand;
import frc.robot.commands.automaticIntakeAndIndexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.RealIndexer;
import frc.robot.subsystems.Indexer.SimIndexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.arm.SimArm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO_Real;
import frc.robot.subsystems.drive.SwerveModuleIO_Sim;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntake;
import frc.robot.subsystems.intake.SimIntake;
import frc.robot.subsystems.shooter.RealShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.SimShooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static LED m_led;
  private DriveSubsystem m_robotDrive;
  private static GyroIO m_gyro;

  public boolean fieldOrientedDrive = true;
  public static boolean isInClimberMode = false;

  // swerve module IOs
  private SwerveModuleIO m_frontLeftIO;
  private SwerveModuleIO m_frontRightIO;
  private SwerveModuleIO m_rearLeftIO;
  private SwerveModuleIO m_rearRightIO;

  // subsystems
  public static Shooter m_shooter;
//   public static Intake m_intake;
  public static Indexer m_indexer;
  public static Climber m_climber;
  public static Arm m_arm;
  public static Vision m_vision;

  // subsystem IOs
  ShooterIO shooterIO;
  //IntakeIO intakeIO;
  IndexerIO indexerIO;
  ArmIO armIO;
  ClimberIO climberIO;
  VisionIO visionIO;

  // auton chooser
  //private static SendableChooser<Command> m_autoChooser;

  // The driver and operator controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setUpSubsystems();
    configureDefaultCommands();
    configureButtonBindingsDriver();
    configureButtonBindingsOperatorClimber();
    configureButtonBindingsOperatorNotClimber();
    //setUpAuton();

  }

//   public Command getAutonomousCommand() {
//     return m_autoChooser.getSelected();
//   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // false, false));
  // }

  private void setUpSubsystems() {

    if (Robot.robotType == RobotType.SIMULATION) {
      indexerIO = new SimIndexer();
      shooterIO = new SimShooter();
      //intakeIO = new SimIntake();
      climberIO = new ClimberSim();
      armIO = new SimArm();
      m_gyro = new GyroIOSim();
      m_frontLeftIO = new SwerveModuleIO_Sim("front left");
      m_frontRightIO = new SwerveModuleIO_Sim("front right");
      m_rearLeftIO = new SwerveModuleIO_Sim("rear left");
      m_rearRightIO = new SwerveModuleIO_Sim("rear right");

      m_robotDrive = new DriveSubsystem(
      new SwerveModule(m_frontLeftIO),
      new SwerveModule(m_frontRightIO),
      new SwerveModule(m_rearLeftIO),
      new SwerveModule(m_rearRightIO), m_gyro);

      visionIO = new VisionSim(m_robotDrive);

    } else {

      m_frontLeftIO = new SwerveModuleIO_Real(DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset,
          "front left");
      m_frontRightIO = new SwerveModuleIO_Real(DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset,
          "front right");
      m_rearLeftIO = new SwerveModuleIO_Real(DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId, DriveConstants.kRearLeftChassisAngularOffset,
          "rear left");
      m_rearRightIO = new SwerveModuleIO_Real(DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId, DriveConstants.kRearRightChassisAngularOffset,
          "rear right");
      
      indexerIO = new RealIndexer();
      shooterIO = new RealShooter();
      //intakeIO = new SimIntake();
      climberIO = new ClimberSim();
      armIO = new RealArm();
      m_gyro = new GyroIOPigeon2();
      visionIO = new VisionReal();

      m_robotDrive = new DriveSubsystem(
      new SwerveModule(m_frontLeftIO),
      new SwerveModule(m_frontRightIO),
      new SwerveModule(m_rearLeftIO),
      new SwerveModule(m_rearRightIO), m_gyro);
     
    }

    m_climber = new Climber(climberIO);
      m_arm = new Arm(armIO);
      m_shooter = new Shooter(shooterIO);
      m_indexer = new Indexer(indexerIO);
    //   m_intake = new Intake(intakeIO);
      m_led = new LED(m_climber);
      m_vision = new Vision(visionIO);
  }

  // sets up auton commands
//   private void setUpAuton() {
//     NamedCommands.registerCommand("intake", Commands.print("intake")); // sensorIntakeCommand());
//     NamedCommands.registerCommand("shoot", Commands.print("/n/n/n/n/n/n/n/nshoot/n/n/n/n/n/n/n/n")); // autoShoot());
//     NamedCommands.registerCommand("AimToTarget", Commands.print("aimed to target!"));
//     NamedCommands.registerCommand("SetArmPosition", Commands.print("set arm position"));
//     m_autoChooser = AutoBuilder.buildAutoChooser();
//     SmartDashboard.putData("Autos/Selector", m_autoChooser);
//   }

  // Configure default commands
  private void configureDefaultCommands() {
    // default command for the shooter: do nothing
    m_shooter.setDefaultCommand(
        new RunCommand(
            () -> m_shooter.setMotor(0),
            m_shooter));

    // default command for intake: do nothing
    // m_intake.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_intake.setMotor(0),
    //         m_intake));

    // default command for indexer: do nothing
    m_indexer.setDefaultCommand(
        new RunCommand(
            () -> m_indexer.setMotor(0),
            m_indexer));

    // default command for climber: do nothing
    m_climber.setDefaultCommand(
        new RunCommand(
            () -> m_climber.setMotors(0),
            m_climber));

    // Arm default command; do nothing but with gravity compensation so it stays
    // where it is.
    // Setpoint is in RADIANS
    m_arm.setEncoderPosition(m_arm.getAbsoluteEncoderPosition());
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.setSpeedGravityCompensation(0), m_arm));

    // default command for drivetrain: drive based on controller inputs
    //actually driving robot
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftY(), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftX(), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_driverController.getRightX(), 3), OIConstants.kDriveDeadband),
                fieldOrientedDrive),
            m_robotDrive));

  }

  private void configureButtonBindingsDriver() {

    // driver left bumper: manual shoot
    //gets arm height to assign to speed. lower arm, means cloesr to speaekr, so shoots less forecfully
    m_driverController.leftBumper().whileTrue(
        new RunCommand(() -> m_shooter.setMotor(m_arm.getSpeedFromArmHeight()), m_shooter));
    

    // driver right bumper: auto-shoot
    m_driverController.rightBumper().onTrue(shootAfterDelay());

    // driver a: automatic intaking
    //m_driverController.rightTrigger().whileTrue(new automaticIntakeAndIndexer(m_indexer,
        //m_intake));

    // driver right trigger: manual intake with arm height restriction
    //only intakes if arm is lowered
    m_driverController.rightTrigger().whileTrue(intakeWithHeightRestriction());

    // driver left trigger: outtake
    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
       //new RunCommand(() -> m_intake.setMotor(-0.3), m_intake),
        new RunCommand(() -> m_indexer.setMotor(-0.3), m_indexer)));

    // driver b: reset gyro
    m_driverController.b().onTrue(new InstantCommand(() -> m_gyro.setYaw(0.0)));

    //driver a: align to speaker mode
    m_driverController.a().whileTrue(
      new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                m_vision.keepPointedAtSpeaker(),
                fieldOrientedDrive), m_robotDrive))
        .onFalse(new RunCommand(() -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            fieldOrientedDrive), m_robotDrive));

    
  }

  private void configureButtonBindingsOperatorClimber() {
    // operater left trigger: climber mode: left climber up
    m_operatorController.leftTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
       () -> m_climber.setLeftSpeed(0.2), m_climber));

    // operater right trigger: climber mode: right climber up
    m_operatorController.rightTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
      () -> m_climber.setRightSpeed(0.2), m_climber));

    // operater left bumper: climber mode: left climber down
    m_operatorController.leftBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
         () -> m_climber.setLeftSpeed(-0.2), m_climber));

    // operater right bumper: climber mode: right climber down
    m_operatorController.rightBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
         () -> m_climber.setRightSpeed(-0.2), m_climber));

    // operator b (climber mode): automatic climber up
    m_operatorController.b().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0.4));

    // operator a (climber mode): automatic climber down
    m_operatorController.a().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0));

    // operator x: switch operator controller modes
    m_operatorController.x().onTrue(new InstantCommand(() -> isInClimberMode = !isInClimberMode));

  }

  private void configureButtonBindingsOperatorNotClimber() {
    // operator right trigger: manual arm up
     m_operatorController.rightTrigger().whileTrue( // TODO: test + change to actual makeSetPositionCommand
        new RunCommand(() -> SmartDashboard.putNumber("arm angle radians", m_vision.keepArmAtAngle())));

    // operator left trigger: manual arm down
     m_operatorController.leftTrigger().and(() -> !isInClimberMode)
         .whileTrue(makeSetSpeedGravityCompensationCommand(m_arm,
             -0.1))
         .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));

    // operater a: arm to intake/subwoofer angle
     m_operatorController.a().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(m_arm, 0.335));

    // operator b: arm to podium shot angle
     m_operatorController.b().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(m_arm, 0.662));

    // operator y: arm to amp angle
    m_operatorController.y().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(m_arm, 1.4));

    // operator left trigger: intake
    m_operatorController.rightBumper().and(() -> !isInClimberMode).whileTrue(new RunCommand(() -> m_indexer.setMotor(0.3), m_indexer));

    // operator right trigger: outtake
    //outtake a little bittt to get shooter up to speed
    m_operatorController.leftBumper().and(() -> !isInClimberMode).onTrue(new RunCommand(() -> m_indexer.setMotor(-0.3), m_indexer).withTimeout(0.1));
  }

  public static Command makeSetPositionCommand(Arm arm,
      double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {
        }), new InstantCommand(() -> arm.enable(), arm), () -> arm.isEnabled()),
        // new InstantCommand(() ->
        // arm.setEncoderPosition(arm.getAbsoluteEncoderPosition())),
        new RunCommand(() -> arm.setGoal(target), arm));
  }

  private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> a.disable(), a),
        new RunCommand(() -> a.setSpeedGravityCompensation(speed), a));
  }

  private Command intakeForTime(Intake intake, Indexer indexer) {
    return new ParallelCommandGroup(
        new RunCommand(() -> intake.setMotor(.8)).withTimeout(1.5),
        new RunCommand(() -> indexer.setMotor(0.8)).withTimeout(1.5));
  }

  private Command setIndexerAndIntakeSpeed(Indexer indexer, Intake intake, double speed) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> intake.setMotor(speed)),
        new InstantCommand(() -> indexer.setMotor(speed)));
  }
//waiting 0.5 seconds to get shooter up to speed
  private Command shootAfterDelay() {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
            new WaitCommand(0.5),
            new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer),
            new RunCommand(() -> m_indexer.setIsIntooked(false), m_indexer)),
        new RunCommand(() -> m_shooter.setMotor(m_arm.getSpeedFromArmHeight()), m_shooter)).withTimeout(0.75);
  }

  private Command outtakeAndShootAfterDelay() {
    return new SequentialCommandGroup(
        //new RunCommand(() -> m_indexer.setMotor(-0.1), m_intake).withTimeout(0.1),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer),
                new RunCommand(() -> m_indexer.setIsIntooked(false), m_indexer)),
            new RunCommand(() -> m_shooter.setMotor(0.8), m_shooter)).withTimeout(1));
  }

  private Command intakeWithHeightRestriction() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            //new RunCommand(() -> m_intake.setMotor(Constants.IntakeConstants.INTAKING_SPEED), m_intake),
            new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer)),
        new ParallelCommandGroup(
            //new RunCommand(() -> m_intake.setMotor(0), m_intake),
            new RunCommand(() -> m_indexer.setMotor(0), m_indexer)),
        () -> m_arm.getEncoderPosition() < 0.35);
  }
}
