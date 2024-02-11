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
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.RealIndexer;
import frc.robot.subsystems.Indexer.SimIndexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.RealArm;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private static Gyro m_gyro = new Gyro();
  // public boolean fieldOrientedDrive = false;
  public static boolean isInClimberMode = false;
  // public static CommandSelector angleHeight = CommandSelector.INTAKE;

  // public static Shooter m_shooter;
  // public static Intake m_intake;
  // public static Indexer m_indexer;
  public static Climber m_climber;
  // ShooterIO shooterIO;
  // IntakeIO intakeIO;
  // IndexerIO indexerIO;
  // ArmIO armIO;
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
    // m_gyro = new Gyro();
    // m_robotDrive = new DriveSubsystem();
    // set up IOs
    // ArmIO armIO;
    // IOs currently always real
    // if (RobotBase.isSimulation()) {
    // indexerIO = new SimIndexer();
    // shooterIO = new SimShooter();
    // intakeIO = new SimIntake();
    // climberIO = new ClimberSim();
    // } else {
    // intakeIO = new RealIntake();
    // indexerIO = new RealIndexer();
    // shooterIO = new RealShooter();
    climberIO = new ClimberReal();
    // }

    // armIO = new RealArm();
    // initialize subsystems
    // m_intake = new Intake(intakeIO);
    // // m_arm = new Arm(armIO);
    // m_shooter = new Shooter(shooterIO);
    // m_indexer = new Indexer(indexerIO);
    m_climber = new Climber(climberIO);
  }

  // Configure default commands
  private void configureDefaultCommands() {
    // default command for the shooter: setting speed to number input from the smart
    // dashboard
    // m_shooter.setDefaultCommand(
    // new InstantCommand(
    // () -> m_shooter.setMotor(SmartDashboard.getNumber("Shoot speed", 0)),
    // m_shooter));

    // // default command for intake: do nothing
    // m_intake.setDefaultCommand(
    // new InstantCommand(
    // () -> m_intake.setMotor(0),
    // m_intake));

    // m_indexer.setDefaultCommand(
    // new InstantCommand(
    // () -> m_indexer.setMotor(0),
    // m_indexer));

    m_climber.setDefaultCommand(
        new InstantCommand(
            () -> m_climber.setMotors(0),
            m_climber));

    // default command for drivetrain: drive based on controller inputs
    // m_robotDrive.setDefaultCommand(
    // // The left stick controls translation of the robot.
    // // Turning is controlled by the X axis of the right stick.
    // new RunCommand(
    // () -> m_robotDrive.drive(
    // -MathUtil.applyDeadband(m_driverController.getLeftY(),
    // OIConstants.kDriveDeadband),
    // -MathUtil.applyDeadband(m_driverController.getLeftX(),
    // OIConstants.kDriveDeadband),
    // -MathUtil.applyDeadband(m_driverController.getRightX(),
    // OIConstants.kDriveDeadband),
    // fieldOrientedDrive, false),
    // m_robotDrive));
    // this version only goes straight (for testing)
    // new RunCommand(
    // () -> m_robotDrive.drive(
    // -MathUtil.applyDeadband(m_driverController.getLeftY(),
    // OIConstants.kDriveDeadband),
    // 0,
    // 0,
    // fieldOrientedDrive, false),
    // m_robotDrive));
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
    // right bumper?
    // new JoystickButton(m_driverController, XboxController.Button.kX.value)
    // .whileTrue(new RunCommand(
    // () -> m_robotDrive.setX(),
    // m_robotDrive));

    // new JoystickButton(m_driverController, XboxController.Button.kY.value)
    // .whileTrue(new RunCommand(
    // () -> m_robotDrive.setZero(),
    // m_robotDrive));

    // m_operatorController.a().onTrue(new InstantCommand(
    // () -> fieldOrientedDrive = !fieldOrientedDrive));
    // // below is old version of button binding
    // // new JoystickButton(m_driverController, XboxController.Button.kA.value)
    // // .onTrue(new InstantCommand(
    // // () -> fieldOrientedDrive = !fieldOrientedDrive));

    // // new JoystickButton(m_driverController, XboxController.Button.kB.value)
    // // .onTrue(new InstantCommand(
    // // () -> m_gyro.resetYaw(), m_gyro));

    // m_driverController.leftBumper().and(() -> !isInClimberMode).whileTrue(new
    // InstantCommand(
    // () -> m_shooter.setMotor(0.8)));

    // below is old version of button binding
    // new JoystickButton(m_driverController,
    // XboxController.Button.kLeftBumper.value)
    // .whileTrue(new InstantCommand(
    // () -> m_shooter.setMotor(0.8)));

    // m_operatorController.rightBumper().and(() -> !isInClimberMode).onTrue(new
    // ParallelCommandGroup(
    // new SequentialCommandGroup(
    // new WaitUntilCommand(() -> m_shooter.getEncoderSpeed() ==
    // Constants.ShooterConstants.speakerSpeed),
    // new InstantCommand(() -> m_intake.setMotor(0.8))),
    // new InstantCommand(() ->
    // m_shooter.setMotor(Constants.ShooterConstants.speakerSpeed))));

    // // change the button binding and finish command
    // m_operatorController.rightTrigger().and(() -> !isInClimberMode).onTrue(new
    // SequentialCommandGroup(
    // new InstantCommand(() -> intakeIO.setMotor(0.8)),
    // new WaitUntilCommand(() -> m_intake.isIntooked()),
    // new InstantCommand(() -> intakeIO.setMotor(0))));

    m_operatorController.leftTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setLeftSpeed(0.2), m_climber)

    );
    m_operatorController.rightTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setRightSpeed(0.2), m_climber)

    );
    m_operatorController.leftBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setLeftSpeed(-0.2), m_climber)

    );
    m_operatorController.rightBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setRightSpeed(-0.2), m_climber)

    );

    m_operatorController.b().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0.4));

    m_operatorController.a().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0));

    m_operatorController.x().onTrue(new InstantCommand(() -> isInClimberMode = !isInClimberMode));

    // m_operatorController.b().and(() -> isInClimberMode).onTrue(new
    // ParallelCommandGroup(
    // new InstantCommand(() -> m_climber.setLeftMotor(ClimberConstants.MAX_HEIGHT -
    // 0.1), m_climber),
    // new InstantCommand(() -> m_climber.setRightMotor(ClimberConstants.MAX_HEIGHT
    // - 0.1)))

    // );
    // m_operatorController.a().and(() -> isInClimberMode).onTrue(new
    // ParallelCommandGroup(
    // new InstantCommand(() -> m_climber.setLeftMotor(ClimberConstants.MIN_HEIGHT +
    // 0.1), m_climber),
    // new InstantCommand(() -> m_climber.setRightMotor(ClimberConstants.MIN_HEIGHT
    // + 0.1)))

    // );

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

  // public static Command makeSetPositionCommand(ProfiledPIDSubsystem base,
  // double target) {
  // return new SequentialCommandGroup(
  // new ConditionalCommand(new InstantCommand(() -> {
  // }), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),
  // new InstantCommand(() -> base.setGoal(target), base));
  // }

  // private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
  // return new SequentialCommandGroup(
  // new InstantCommand(() -> a.disable()),
  // new RunCommand(() -> a.setSpeedGravityCompensation(speed), a));
  // }

  // static final double DELAY_OVERHEAD_SECONDS = 0.5;
  // static final double correctSpeed = 0.8;

  // set motor to speed, if statement to check if shooter is at correct speed and
  // set indexer to speed

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand = new
  // SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

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

  // public enum CommandSelector {
  // INTAKE,
  // AMP,
  // SPEAKER_SUBWOOFER_STRAIGHT,
  // SPEAKER_SUBWOOFER_SIDE,
  // SPEAKER_PODIUM
  // }

  // private CommandSelector select() {
  // return angleHeight;
  // }

  // public static String toString(CommandSelector node) {
  // return "Node: " + node;
  // }

  // private Command selectPositionCommand() {
  // return new SelectCommand(
  // Map.ofEntries(
  // Map.entry(CommandSelector.INTAKE, makeSetPositionCommand(m_arm,
  // ArmConstants.INTAKE_ANGLE)),
  // Map.entry(CommandSelector.AMP, makeSetPositionCommand(m_arm,
  // ArmConstants.AMP_ANGLE)),
  // Map.entry(CommandSelector.SPEAKER_SUBWOOFER_STRAIGHT,makeSetPositionCommand(m_arm,
  // ArmConstants.SPEAKER_SUBWOOFER_STRAIGHT_ANGLE)),
  // Map.entry(CommandSelector.SPEAKER_SUBWOOFER_SIDE,
  // makeSetPositionCommand(m_arm, ArmConstants.SPEAKER_SUBWOOFER_SIDE_ANGLE)),
  // Map.entry(CommandSelector.SPEAKER_PODIUM, makeSetPositionCommand(m_arm,
  // ArmConstants.SPEAKER_PODIUM_ANGLE))),
  // this::select);
  // }
}
