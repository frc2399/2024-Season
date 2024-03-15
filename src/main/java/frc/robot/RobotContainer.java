// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot.RobotType;
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
  public static Intake m_intake;
  public static Indexer m_indexer;
  public static Climber m_climber;
  public static Arm m_arm;
  public static Vision m_vision;

  // subsystem IOs
  ShooterIO shooterIO;
  IntakeIO intakeIO;
  IndexerIO indexerIO;
  ArmIO armIO;
  ClimberIO climberIO;
  VisionIO visionIO;

  // auton chooser
  private static SendableChooser<Command> m_autoChooser;
  ComplexWidget autonChooserWidget;

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
    setUpAuton();

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private void setUpSubsystems() {

    if (Robot.robotType == RobotType.SIMULATION) {
      indexerIO = new SimIndexer();
      shooterIO = new SimShooter();
      intakeIO = new SimIntake();
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
      intakeIO = new RealIntake();
      climberIO = new ClimberReal();
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
    m_intake = new Intake(intakeIO);
    m_led = new LED(m_climber);
    m_vision = new Vision(visionIO);
  }

  // sets up auton commands
  private void setUpAuton() {
    NamedCommands.registerCommand("intake", intakeWithHeightRestriction());
    NamedCommands.registerCommand("Intake", intakeWithHeightRestriction());
    NamedCommands.registerCommand("intake for time", intakeForTime(m_intake, m_indexer));
    NamedCommands.registerCommand("SHORT intake for time", shortIntakeForTime(m_intake, m_indexer));
    NamedCommands.registerCommand("AimToTarget", Commands.print("aimed to target!"));
    NamedCommands.registerCommand("SetArmPosition", makeSetPositionCommandAuton(m_arm, 0.475));
    NamedCommands.registerCommand("Set Arm Wingleft", makeSetPositionCommandAuton(m_arm, 0.745));
    NamedCommands.registerCommand("SetArmDown", makeSetPositionCommandAuton(m_arm, 0.335));
    NamedCommands.registerCommand("AutoShoot", outtakeAndShootAfterDelay());
    NamedCommands.registerCommand("intake and outtake", intakeAndOuttake());
    NamedCommands.registerCommand("outtake", outtake());

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos/Selector", m_autoChooser);

  }

  // Configure default commands
  private void configureDefaultCommands() {
    // default command for the shooter: do nothing
    m_shooter.setDefaultCommand(
        new RunCommand(
            () -> m_shooter.setMotor(0),
            m_shooter).withName("drive default"));

    // default command for intake: do nothing
    m_intake.setDefaultCommand(
        new RunCommand(
            () -> m_intake.setMotor(0),
            m_intake).withName("drive default"));

    // default command for indexer: do nothing
    m_indexer.setDefaultCommand(
        new RunCommand(
            () -> m_indexer.setMotor(0),
            m_indexer).withName("drive default"));

    // default command for climber: do nothing
    m_climber.setDefaultCommand(
        new RunCommand(
            () -> m_climber.setMotors(0),
            m_climber));

    // Arm default command; do nothing but with gravity compensation so it stays
    // where it is.
    // Setpoint is in RADIANS
    m_arm.setEncoderPosition(m_arm.getAbsoluteEncoderPosition());
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.setSpeedGravityCompensation(0), m_arm).withName("drive default"));

    // default command for drivetrain: drive based on controller inputs
    // actually driving robot
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -Math.pow(MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 3),
                -Math.pow(MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 3),
                -Math.pow(MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 3),
                fieldOrientedDrive),
            m_robotDrive).withName("drive default"));

  }

  private void configureButtonBindingsDriver() {
    // while true with run commands
    m_driverController.y().whileTrue((new RunCommand(() -> m_robotDrive.setZero(), m_robotDrive).withName("setzero")));
    m_driverController.x().whileTrue((new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive).withName("setx")));


    // driver left bumper: manual shoot
    // gets arm height to assign to speed. lower arm, means cloesr to speaekr, so
    // shoots less forecfully
    m_driverController.leftBumper().whileTrue(
        new RunCommand(() -> m_shooter.setMotor(m_arm.getSpeedFromArmHeight()), m_shooter));

    // driver right bumper: auto-shoot
    m_driverController.rightBumper().onTrue(shootAfterDelay());

    // driver right trigger: manual intake with arm height restriction
    // only intakes if arm is lowered
    m_driverController.rightTrigger().whileTrue(intakeWithHeightRestriction());

    // driver left trigger: outtake
    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
        new RunCommand(() -> m_intake.setMotor(-0.3), m_intake),
        new RunCommand(() -> m_indexer.setMotor(-0.3), m_indexer)));

    // driver b: reset gyro
    m_driverController.b().onTrue(new InstantCommand(() -> m_gyro.setYaw(0.0)));

    // driver a: align to speaker mode
    m_driverController.a().whileTrue(
        new RunCommand(() -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            m_vision.keepPointedAtSpeaker(),
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
    m_operatorController.b().and(() -> isInClimberMode).whileTrue(new RunCommand(() -> m_climber.setMotors(0.5)));

    // operator a (climber mode): automatic climber down
    m_operatorController.a().and(() -> isInClimberMode).whileTrue(new RunCommand(() -> m_climber.setMotors(-0.5)));

    // operator x: switch operator controller modes
    m_operatorController.x().onTrue(new InstantCommand(() -> isInClimberMode = !isInClimberMode));

  }

  private void configureButtonBindingsOperatorNotClimber() {
    m_operatorController.rightTrigger().and(() -> !isInClimberMode).whileTrue( // TODO: test + change to actual
                                                                               // makeSetPositionCommand
        makeSetPositionCommand(m_arm, m_vision.keepArmAtAngle()));

    // operator left trigger: manual arm down
    m_operatorController.leftTrigger().and(() -> !isInClimberMode).and(() -> !isInClimberMode)
        .whileTrue(makeSetSpeedGravityCompensationCommand(m_arm, -0.1))
        .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));

    // operater a: arm to intake/subwoofer angle
    m_operatorController.a().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(m_arm, 0.31));

    // operator b: arm to podium shot angle
    m_operatorController.b().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(m_arm, 0.66));

    // operator y: arm to amp angle
    m_operatorController.y().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(m_arm, 1.58));

    // operator left trigger: intake
    m_operatorController.rightBumper().and(() -> !isInClimberMode)
        .whileTrue(new RunCommand(() -> m_indexer.setMotor(0.3), m_indexer));
        

    // operator right trigger: outtake
    // outtake a little bittt to get shooter up to speed
    m_operatorController.leftBumper().and(() -> !isInClimberMode)
        .onTrue(new RunCommand(() -> m_indexer.setMotor(-0.3), m_indexer).withTimeout(0.1));

        //TODO test this!
    m_operatorController.axisGreaterThan(5, 0.1)
        .whileTrue(new RunCommand(() -> m_arm.setSpeedGravityCompensation(0.1), m_arm));
    m_operatorController.axisLessThan(5, -0.1)
        .whileTrue(new RunCommand(() -> m_arm.setSpeedGravityCompensation(-0.1), m_arm));
  }

  public static Command makeSetPositionCommand(Arm arm,
      double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {
        }), new InstantCommand(() -> arm.enable(), arm), () -> arm.isEnabled()),
        new RunCommand(() -> arm.setGoal(target), arm));
  }

  public static Command makeSetPositionCommandAuton(Arm arm,
      double target) {
    return new SequentialCommandGroup(
        new ConditionalCommand(new InstantCommand(() -> {
        }), new InstantCommand(() -> arm.enable(), arm), () -> arm.isEnabled()),
        new InstantCommand(() -> arm.setGoal(target), arm),
        new WaitCommand(0.5));
  }

  private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> a.disable(), a),
        new RunCommand(() -> a.setSpeedGravityCompensation(speed), a));
  }

  private Command intakeForTime(Intake intake, Indexer indexer) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new RunCommand(() -> intake.setMotor(.8), intake),
            new RunCommand(() -> indexer.setMotor(0.8), indexer)
        ).withTimeout(1.5)
     );
  }

  private Command shortIntakeForTime(Intake intake, Indexer indexer) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> intake.setMotor(.8)).withTimeout(0.2),
            new InstantCommand(() -> indexer.setMotor(0.8)).withTimeout(0.2)),
        new ParallelCommandGroup(
            new InstantCommand(() -> intake.setMotor(0)),
            new InstantCommand(() -> indexer.setMotor(0))));

  }

  private Command setIndexerAndIntakeSpeed(Indexer indexer, Intake intake, double speed) {
    return new ParallelCommandGroup(
        new RunCommand(() -> intake.setMotor(speed)),
        new RunCommand(() -> indexer.setMotor(speed))).withTimeout(0.4);
  }

  // waiting 0.5 seconds to get shooter up to speed
  private Command shootAfterDelay() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer),
                new RunCommand(() -> m_indexer.setIsIntooked(false), m_indexer)),
            new RunCommand(() -> m_shooter.setMotor(m_arm.getSpeedFromArmHeight()), m_shooter)).withTimeout(0.75),
        new InstantCommand(() -> m_shooter.setMotor(0), m_shooter),
        new InstantCommand(() -> m_indexer.setMotor(0), m_indexer));

  }

  private Command outtakeAndShootAfterDelay() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer),
                new RunCommand(() -> m_indexer.setIsIntooked(false), m_indexer)
                ),
            new RunCommand(() -> m_shooter.setMotor(m_arm.getSpeedFromArmHeight()), m_shooter)).
                 withTimeout(0.5),
        new InstantCommand(() -> m_shooter.setMotor(0), m_shooter),
        new InstantCommand(() -> m_indexer.setMotor(0), m_indexer)
        );
  }

  private Command intakeWithHeightRestriction() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new RunCommand(() -> m_intake.setMotor(Constants.IntakeConstants.INTAKING_SPEED), m_intake),
            new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer)),
        new ParallelCommandGroup(
            new RunCommand(() -> m_intake.setMotor(0), m_intake),
            new RunCommand(() -> m_indexer.setMotor(0), m_indexer)),
        () -> m_arm.getEncoderPosition() < 0.5); 
  }

  private Command intakeAndOuttake() {
    return new SequentialCommandGroup(
        new RunCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer)
            .withTimeout(0.2),
        new RunCommand(() -> m_indexer.setMotor(-0.05), m_indexer).withTimeout(0.1));
  }

  private Command outtake() {
     return new SequentialCommandGroup(
        new RunCommand(() -> m_indexer.setMotor(-0.15), m_indexer)
            .withTimeout(0.3),
        new RunCommand(() -> m_indexer.setMotor(0), m_indexer).withTimeout(0.1));
  }
}
