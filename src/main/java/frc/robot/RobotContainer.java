// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.automaticClimberCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.List;
import java.util.Map;

import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.arm.SimArm;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.RealIndexer;
import frc.robot.subsystems.Indexer.SimIndexer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntake;
import frc.robot.subsystems.intake.SimIntake;
import frc.robot.subsystems.shooter.RealShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.SimShooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private DriveSubsystem m_robotDrive;
  // private static Gyro m_gyro = new Gyro();
 public boolean fieldOrientedDrive = false;
  public static boolean isInClimberMode = false;
  // public static CommandSelector angleHeight = CommandSelector.INTAKE;

  public static Shooter m_shooter;
  public static Intake m_intake;
  public static Indexer m_indexer;
  public static Climber m_climber;
  public static Arm m_arm;
  ShooterIO shooterIO;
  IntakeIO intakeIO;
  IndexerIO indexerIO;
  ArmIO armIO;
  ClimberIO climberIO;

  // public static Intake m_intake;
  // public static Arm m_arm;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    SmartDashboard.putNumber("Shoot speed", SmartDashboard.getNumber("Shoot speed", 0));
    setUpSubsystems();
    configureDefaultCommands();
    configureButtonBindings();
  }

  // construct subsystems
  private void setUpSubsystems() {
    m_robotDrive = new DriveSubsystem();
    // set up IOs
    if (RobotBase.isSimulation()) {
      indexerIO = new SimIndexer();
      shooterIO = new SimShooter();
      intakeIO = new SimIntake();
      climberIO = new ClimberSim();
      armIO = new SimArm();
    } else {
      intakeIO = new RealIntake();
      indexerIO = new RealIndexer();
      shooterIO = new RealShooter();
      climberIO = new ClimberReal();
      armIO = new RealArm();
    }

    // armIO = new RealArm();
    // initialize subsystems
    // m_intake = new Intake(intakeIO);
    // // m_arm = new Arm(armIO);
    // m_shooter = new Shooter(shooterIO);
    // m_indexer = new Indexer(indexerIO);
    m_climber = new Climber(climberIO);
    m_arm = new Arm(armIO);
    m_shooter = new Shooter(shooterIO);
    m_indexer = new Indexer(indexerIO);
    m_intake = new Intake(intakeIO);
  }

  // Configure default commands
  private void configureDefaultCommands() {
    // default command for the shooter: setting speed to number input from the smart
    // dashboard
    m_shooter.setDefaultCommand(
    new InstantCommand(
    () -> m_shooter.setMotor(SmartDashboard.getNumber("Shoot speed", 0)),
    m_shooter));

    // default command for intake: do nothing
    m_intake.setDefaultCommand(
    new InstantCommand(
    () -> m_intake.setMotor(0),
    m_intake));

    m_indexer.setDefaultCommand(
    new InstantCommand(
    () -> m_indexer.setMotor(0),
    m_indexer));

    m_climber.setDefaultCommand(
        new InstantCommand(
            () -> m_climber.setMotors(0),
            m_climber));
    
    m_arm.setDefaultCommand(new InstantCommand(() -> m_arm.setSpeedGravityCompensation(0), m_arm));
    //Arm setpoint must be in RADIANS
    //m_arm.setDefaultCommand(makeSetPositionCommand(m_arm, Units.degreesToRadians(SmartDashboard.getNumber("arm setpoint", 0))));
    // default command for drivetrain: drive based on controller inputs
    m_robotDrive.setDefaultCommand(
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    new RunCommand(
    () -> m_robotDrive.drive(
    -MathUtil.applyDeadband(m_driverController.getLeftY(),
    OIConstants.kDriveDeadband),
    -MathUtil.applyDeadband(m_driverController.getLeftX(),
    OIConstants.kDriveDeadband),
    -MathUtil.applyDeadband(m_driverController.getRightX(),
    OIConstants.kDriveDeadband),
    fieldOrientedDrive, false),
    m_robotDrive));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_operatorController.povCenter().onTrue(new InstantCommand(
        () -> fieldOrientedDrive = !fieldOrientedDrive));

    //driver left bumper: manual shoot
    m_driverController.leftBumper().and(() -> !isInClimberMode).whileTrue(new SequentialCommandGroup(
        new InstantCommand(() -> m_shooter.setMotor(0.8))));

    //driver right bumper: auto-shoot
    m_driverController.rightBumper().and(() -> !isInClimberMode).onTrue(new ParallelCommandGroup(
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_shooter.getEncoderSpeed() >= Constants.ShooterConstants.speakerSpeed - 0.05),
          new InstantCommand(() -> m_intake.setMotor(0.8))),
      new InstantCommand(() -> m_shooter.setMotor(Constants.ShooterConstants.speakerSpeed))));

    //driver right trigger: intake
    m_driverController.rightTrigger().whileTrue(sensorIntakeCommand()
    );

    //driver left trigger: outtake
    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> m_intake.setMotor(-0.5), m_intake), 
      new RunCommand(() -> m_indexer.setMotor(-0.5), m_indexer))
    );

    //operater left trigger: climber mode: left climber up
    m_operatorController.leftTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setLeftSpeed(0.2), m_climber)

    );

    //operater right trigger: climber mode: right climber up
    m_operatorController.rightTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setRightSpeed(0.2), m_climber)

    );

    //operater left bumper: climber mode: left climber down
    m_operatorController.leftBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setLeftSpeed(-0.2), m_climber)

    );

    //operater right bumper: climber mode: right climber down
    m_operatorController.rightBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setRightSpeed(-0.2), m_climber)

    );

    //operator x: switch operator controller modes
    m_operatorController.x().onTrue(new InstantCommand(() -> isInClimberMode = !isInClimberMode, m_climber));

    m_operatorController.b().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0.4));

    m_operatorController.a().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0));


    // Right Y axis to control the arm
    // // TODO: is this accurate? could totally be the wrong axis.
    // m_operatorController.axisGreaterThan(5, 0.1).and(() -> !isInClimberMode).whileTrue(makeSetSpeedGravityCompensationCommand(m_arm, 0.1))
    //     .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));
    // m_operatorController.axisLessThan(5, -0.1).and(() -> !isInClimberMode).whileTrue(makeSetSpeedGravityCompensationCommand(m_arm, -0.1))
    //     .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));
    
    
    // m_driverController.a().onTrue(setkG(arm, SmartDashboard.getNumber("kG", 0)));
    // new JoystickButton(m_driverController, XboxController.Button.kB.value)
    // .onTrue(new InstantCommand(
    // () -> m_gyro.resetYaw(), m_gyro));

    m_operatorController.a().and(() -> isInClimberMode).onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_climber.setLeftMotor(ClimberConstants.MIN_HEIGHT + 0.1), m_climber),
        new InstantCommand(() -> m_climber.setRightMotor(ClimberConstants.MIN_HEIGHT + 0.1)))

    );

    m_operatorController.rightTrigger().and(() -> !isInClimberMode).whileTrue(new ParallelCommandGroup(
      new InstantCommand(() -> m_intake.setMotor(Constants.IntakeConstants.INTAKING_SPEED)),
      new InstantCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED))
    ));

    m_operatorController.leftTrigger().and(() -> !isInClimberMode).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.setMotor(Constants.IntakeConstants.OUTTAKING_SPEED)),
        new InstantCommand(() -> m_indexer.setMotor(Constants.IndexerConstants.INDEXER_OUTTAKING_SPEED))),
      new InstantCommand(() -> m_indexer.setIsIntooked(false))
    ));

    //randomly assigned button, change as necessary
    m_operatorController.y().and(() -> !isInClimberMode).onTrue(new InstantCommand(
    () -> m_shooter.setMotor(Constants.ShooterConstants.speakerSpeed))
    
    );
    m_operatorController.povCenter().and(() -> !isInClimberMode).onTrue(new InstantCommand(
    () -> m_indexer.setIsIntooked(!m_indexer.isIntooked))
    
    );
    
    // below is old version of button binding
    // run shooter, wait until shooter reaches set speed, run intake to feed shooter
    // new JoystickButton(m_operatorController,
    // XboxController.Button.kRightBumper.value)
    // .onTrue(new ParallelCommandGroup(
    // new SequentialCommandGroup(
    // new WaitUntilCommand(() -> m_shooter.getEncoderSpeed() ==
    // Constants.ShooterConstants.speakerSpeed),
    // new InstantCommand(() -> intakeIO.setMotor(0.8))),
    // new InstantCommand(() ->
    // m_shooter.setMotor(Constants.ShooterConstants.speakerSpeed))
    // ));

    // Left trigger to intake
    // new Trigger(() -> m_driverController.getRawAxis(Axis.kLeftTrigger.value) >
    // 0.1)
    // .whileTrue(new InstantCommand(
    // () -> m_intake.setMotor(1)));

    // //Right trigger to outtake
    // new Trigger(() -> m_driverController.getRawAxis(Axis.kRightTrigger.value) >
    // 0.1)
    // .whileTrue(new InstantCommand(
    // () -> m_intake.setMotor(-1)));

    // //Right stick up to move arm up
    // new Trigger(() -> m_driverController.getRawAxis(Axis.kRightY.value) < -0.1)
    // .whileTrue(makeSetSpeedGravityCompensationCommand(m_arm, 0.2))
    // .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));

    // //Right stick down to move arm down
    // new Trigger(() -> m_driverController.getRawAxis(Axis.kRightY.value) > 0.1)
    // .whileTrue(makeSetSpeedGravityCompensationCommand(m_arm, -0.2))
    // .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));
  }

  public static Command makeSetPositionCommand(ProfiledPIDSubsystem base,
  double target) {
  return new SequentialCommandGroup(
  new ConditionalCommand(new InstantCommand(() -> {
  }), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),
  new InstantCommand(() -> base.setGoal(target), base));
  }

  private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
  return new SequentialCommandGroup(
  new InstantCommand(() -> a.disable()),
  new RunCommand(() -> a.setSpeedGravityCompensation(speed), a));
  }

  private Command setIntakeSpeed (Intake i, double speed) {
    return new InstantCommand(() -> i.setMotor(speed));
  }

  private Command setIndexerSpeed (Indexer i, double speed) {
    return new InstantCommand(() -> i.setMotor(speed));
  }

  private Command setIndexerAndIntakeSpeed (Indexer indexer, Intake intake, double speed) {
    return new ParallelCommandGroup(
      new InstantCommand(() -> intake.setMotor(speed)),
      new InstantCommand(()-> indexer.setMotor(speed)));
  }

  private Command autoShoot () {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_indexer.setIsOverride(true)),
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_shooter.getEncoderSpeed() >= Constants.ShooterConstants.speakerSpeed - 0.05),
          new InstantCommand(() -> m_intake.setMotor(0.8))),
      new InstantCommand(() -> m_shooter.setMotor(Constants.ShooterConstants.speakerSpeed)));
  }

  private Command sensorIntakeCommand() {
    return new SequentialCommandGroup(
      //Tells if override is activated if the speed is greater than 5%
      new ConditionalCommand(new InstantCommand(() -> m_indexer.setIsOverride(true)), new InstantCommand(() ->m_indexer.setIsOverride(false)), () -> m_shooter.getEncoderSpeed() > 0.05),
      //If beam is broken, then the intake and indexer speed is set to zero. If not, then they keep running 50%
      new ConditionalCommand(
        new ParallelCommandGroup(
          new InstantCommand(() -> m_indexer.setMotor(0)),
          new InstantCommand(() -> m_intake.setMotor(0))
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> m_indexer.setMotor(0.5)),
          new InstantCommand(() -> m_intake.setMotor(0.5))
        ), 
       () -> m_indexer.getIsBeamBroken())
      );
    }

  

  // static final double DELAY_OVERHEAD_SECONDS = 0.5;
  // static final double correctSpeed = 0.8;

  // public enum CommandSelector {
  //   INTAKE,
  //   AMP,
  //   SPEAKER_SUBWOOFER_STRAIGHT,
  //   SPEAKER_SUBWOOFER_SIDE,
  //   SPEAKER_PODIUM
  // }

  // private CommandSelector select() {
  //   return angleHeight;
  // }

  // public static String toString(CommandSelector node) {
  //   return "Node: " + node;
  // }
  }
