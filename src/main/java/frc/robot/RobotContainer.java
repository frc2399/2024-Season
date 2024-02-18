// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot.RobotType;
import frc.robot.commands.AimAtTargetCommand;
import frc.robot.commands.AlignAprilTag;
import frc.robot.commands.automaticClimberCommand;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.SimIndexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.SimArm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
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
  private DriveSubsystem m_robotDrive;
  private static GyroIO m_gyro;

  public boolean fieldOrientedDrive = true;
  public static boolean isInClimberMode = false;

      private SwerveModuleIO m_frontLeftIO;
      private SwerveModuleIO m_frontRightIO;
      private SwerveModuleIO m_rearLeftIO;
      private SwerveModuleIO m_rearRightIO;

        private VisionIO m_vision;
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


  //auton chooser
      private static SendableChooser<Command> m_autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                SmartDashboard.putNumber("Shoot speed", SmartDashboard.getNumber("Shoot speed", 0));
                setUpSubsystems();
                m_autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autos/Selector", m_autoChooser);
                // Configure the button bindings
                configureDefaultCommands();
                configureButtonBindings();
                setUpAuton();


                // Configure default commands
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
                                        fieldOrientedDrive),
                                                m_robotDrive));
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
        

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return m_autoChooser.getSelected();
        }
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

                        m_gyro = new GyroIOSim();

                } else {
                        m_gyro = new GyroIOPigeon2();
                        m_frontLeftIO = new SwerveModuleIO_Real(DriveConstants.kFrontLeftDrivingCanId,
                                        DriveConstants.kFrontLeftTurningCanId,
                                        DriveConstants.kFrontLeftChassisAngularOffset,
                                        "front left");
                        m_frontRightIO = new SwerveModuleIO_Real(DriveConstants.kFrontRightDrivingCanId,
                                        DriveConstants.kFrontRightTurningCanId,
                                        DriveConstants.kFrontRightChassisAngularOffset,
                                        "front right");
                        m_rearLeftIO = new SwerveModuleIO_Real(DriveConstants.kRearLeftDrivingCanId,
                                        DriveConstants.kRearLeftTurningCanId,
                                        DriveConstants.kRearLeftChassisAngularOffset,
                                        "rear left");
                        m_rearRightIO = new SwerveModuleIO_Real(DriveConstants.kRearRightDrivingCanId,
                                        DriveConstants.kRearRightTurningCanId,
                                        DriveConstants.kRearRightChassisAngularOffset,
                                        "rear right");
                }
                m_robotDrive = new DriveSubsystem(
                                new SwerveModule(m_frontLeftIO),
                                new SwerveModule(m_frontRightIO),
                                new SwerveModule(m_rearLeftIO),
                                new SwerveModule(m_rearRightIO), m_gyro);

                                if (Robot.robotType == RobotType.SIMULATION) {
                        m_vision = new VisionSim(m_robotDrive);
                } else {
                        m_vision = new VisionReal();
                }
        }

        
  private void setUpAuton(){
        NamedCommands.registerCommand("intake", Commands.print("intake")); //sensorIntakeCommand());
        NamedCommands.registerCommand("shoot", Commands.print("shoot")); //autoShoot());
        NamedCommands.registerCommand("AimToTarget", Commands.print("aimed to target!"));
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autos/Selector", m_autoChooser);
  }
    // Configure default commands
    private void configureDefaultCommands() {
        // default command for the shooter: setting speed to number input from the smart
        // dashboard
        m_shooter.setDefaultCommand(
            new RunCommand(
                () -> m_shooter.setMotor(0),
                m_shooter));
    
        // default command for intake: do nothing
        m_intake.setDefaultCommand(
            new RunCommand(
                () -> m_intake.setMotor(0),
                m_intake));
    
        m_indexer.setDefaultCommand(
            new RunCommand(
                () -> m_indexer.setMotor(0),
                m_indexer));
    
        m_climber.setDefaultCommand(
            new RunCommand(
                () -> m_climber.setMotors(0),
                m_climber));
    
        m_arm.setDefaultCommand(new InstantCommand(() -> m_arm.setSpeedGravityCompensation(0), m_arm));
        // Arm setpoint must be in RADIANS
        // m_arm.setDefaultCommand(makeSetPositionCommand(m_arm,
        // Units.degreesToRadians(SmartDashboard.getNumber("arm setpoint", 0))));
        // default command for drivetrain: drive based on controller inputs
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                    fieldOrientedDrive),
                            m_robotDrive));
    
      }

        public class aprilTagAssignment { 
                public static int facingSourceLeftID;
                public static int facingSourceRightID;
                public static int speakerID;
                public static int speakerOffsetID;
                public static int stageBackID;
                public static int facingAwayFromSpeakerStageLeftID;
                public static int facingAwayFromSpeakerStageRightID;
                public static int ampID;
  
                static void assignAprilTags() {
                if (VisionConstants.isBlueAlliance) {
                        facingSourceLeftID = 1;
                        facingSourceRightID = 2;
                        speakerID = 7;
                        speakerOffsetID = 8;
                        stageBackID = 14;
                        facingAwayFromSpeakerStageLeftID = 15;
                        facingAwayFromSpeakerStageRightID = 16;
                        ampID = 6;
                } else {
                        facingSourceLeftID = 10;
                        facingSourceRightID = 9;
                        speakerID = 4;
                        speakerOffsetID = 3;
                        stageBackID = 13;
                        facingAwayFromSpeakerStageLeftID = 11;
                        facingAwayFromSpeakerStageRightID = 12;
                        ampID = 5;
                        }}
                }

                  private void configureButtonBindings() {
    m_operatorController.povCenter().onTrue(new InstantCommand(
        () -> fieldOrientedDrive = !fieldOrientedDrive));

    // driver left bumper: manual shoot
    m_driverController.leftBumper().whileTrue(
      //new SequentialCommandGroup(
        //new InstantCommand(() -> m_indexer.setIsIntooked(false)),
        new RunCommand(() -> m_shooter.setMotor(0.7)))
        //)
        ;

    // driver right bumper: auto-shoot
    m_driverController.rightBumper().and(() -> !isInClimberMode).onTrue(autoShoot());

    // driver right trigger: intake
    //m_driverController.rightTrigger().whileTrue(new automaticIntakeAndIndexer(m_indexer, m_intake)).onFalse(setIndexerAndIntakeSpeed(m_indexer, m_intake, 0));

    m_driverController.rightTrigger().whileTrue(new ParallelCommandGroup(
        new RunCommand(() -> m_intake.setMotor(0.8), m_intake),
        new RunCommand(() -> m_indexer.setMotor(0.8), m_indexer)));

    
    // driver left trigger: outtake
    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
        new RunCommand(() -> m_intake.setMotor(-0.3), m_intake),
        new RunCommand(() -> m_indexer.setMotor(-0.3), m_indexer)));

    m_driverController.b().onTrue(new InstantCommand(() -> m_gyro.setYaw(0.0)));

    m_driverController.a().whileTrue(
        new ParallelCommandGroup(
            new RunCommand(() -> 
            m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            m_vision.keepPointedAtSpeaker(aprilTagAssignment.speakerID),
            fieldOrientedDrive), m_robotDrive), 
            makeSetPositionCommand(m_arm, m_vision.keepPointedAtSpeaker(aprilTagAssignment.speakerID)))).
        onFalse(new RunCommand(() -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
          fieldOrientedDrive), m_robotDrive));

          //make parallel command to also adjust arm height
          //another function in vision
          //will commandeer the arm but only when drivetrain is also yoinked
          //runcommand
   
    // operater left trigger: climber mode: left climber up
    m_operatorController.leftTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setLeftSpeed(0.2), m_climber)

    );

    // operater right trigger: climber mode: right climber up
    m_operatorController.rightTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setRightSpeed(0.2), m_climber)

    );

    // operater left bumper: climber mode: left climber down
    m_operatorController.leftBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setLeftSpeed(-0.2), m_climber)

    );

    // operater right bumper: climber mode: right climber down
    m_operatorController.rightBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
        () -> m_climber.setRightSpeed(-0.2), m_climber)

    );

    // operator x: switch operator controller modes
    m_operatorController.x().onTrue(new InstantCommand(() -> isInClimberMode = !isInClimberMode, m_climber));

    m_operatorController.b().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0.4));

    m_operatorController.a().and(() -> isInClimberMode).onTrue(new automaticClimberCommand(m_climber, 0));


    m_operatorController.rightTrigger().and(() ->
    !isInClimberMode).whileTrue(makeSetSpeedGravityCompensationCommand(m_arm,
    0.1))
    .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));
    
    m_operatorController.leftTrigger().and(() ->
    !isInClimberMode).whileTrue(makeSetSpeedGravityCompensationCommand(m_arm,
    -0.1))
    .onFalse(makeSetSpeedGravityCompensationCommand(m_arm, 0));

    // m_driverController.a().onTrue(setkG(arm, SmartDashboard.getNumber("kG", 0)));


    m_operatorController.a().and(() -> isInClimberMode).onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_climber.setLeftMotor(ClimberConstants.MIN_HEIGHT + 0.1), m_climber),
        new InstantCommand(() -> m_climber.setRightMotor(ClimberConstants.MIN_HEIGHT + 0.1)))

    );

    m_operatorController.povCenter().and(() -> !isInClimberMode).onTrue(new InstantCommand(
        () -> m_indexer.setIsSensorOverriden(!m_indexer.getIsSensorOverriden())));
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

  private Command setIntakeSpeed(Intake i, double speed) {
    return new InstantCommand(() -> i.setMotor(speed));
  }

  private Command setIndexerSpeed(Indexer i, double speed) {
    return new InstantCommand(() -> i.setMotor(speed));
  }

  private Command setIndexerAndIntakeSpeed(Indexer indexer, Intake intake, double speed) {
    return new ParallelCommandGroup(
        new InstantCommand(() -> intake.setMotor(speed)),
        new InstantCommand(() -> indexer.setMotor(speed)));
  }

  private Command autoShoot () {
    return new SequentialCommandGroup(new ParallelCommandGroup(
      new InstantCommand(() -> m_indexer.setIsIntooked(false)),
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_shooter.getEncoderSpeed() >= Constants.ShooterConstants.speakerSpeed - 0.05),
          new InstantCommand(() -> m_indexer.setMotor(0.8))),
      new InstantCommand(() -> m_shooter.setMotor(Constants.ShooterConstants.speakerSpeed))),
      new WaitCommand(1),
      new InstantCommand(() -> m_indexer.setMotor(0)),
      new InstantCommand(() -> m_shooter.setMotor(0)));
  }

  private Command sensorIntakeCommand() {
    return new SequentialCommandGroup(
        // Tells if override is activated if the speed is greater than 5%
        new ConditionalCommand(new InstantCommand(() -> m_indexer.setIsSensorOverriden(true)),
            new InstantCommand(() -> m_indexer.setIsSensorOverriden(false)), () -> m_shooter.getEncoderSpeed() > 0.05),
        // If beam is broken, then the intake and indexer speed is set to zero. If not,
        // then they keep running 50%
        new ConditionalCommand(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_indexer.setMotor(0)),
                new InstantCommand(() -> m_intake.setMotor(0))),
            new ParallelCommandGroup(
                new InstantCommand(() -> m_indexer.setMotor(0.5)),
                new InstantCommand(() -> m_intake.setMotor(0.5))),
            () -> m_indexer.getIsBeamBroken()));
  }

}