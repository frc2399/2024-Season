// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot.RobotType;
import frc.robot.commands.AimAtTargetCommand;
import frc.robot.commands.AlignAprilTag;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO_Real;
import frc.robot.subsystems.drive.SwerveModuleIO_Sim;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private DriveSubsystem m_robotDrive;
        private GyroIO m_gyro;
        public boolean fieldOrientedDrive = true;

        private SwerveModuleIO m_frontLeftIO;
        private SwerveModuleIO m_frontRightIO;
        private SwerveModuleIO m_rearLeftIO;
        private SwerveModuleIO m_rearRightIO;

        private VisionIO m_vision;
        private Vision vision;

        private static SendableChooser<Command> m_autoChooser;

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                setUpSubsystems();
                m_autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autos/Selector", m_autoChooser);
                // Configure the button bindings
                configureButtonBindings();

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
                                        fieldOrientedDrive, false),
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
        private void configureButtonBindings() {
                // right bumper?
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .onTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .onTrue(new RunCommand(
                                                () -> m_robotDrive.setZero(),
                                                m_robotDrive));

                new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(
                                new InstantCommand(
                                                () -> fieldOrientedDrive = !fieldOrientedDrive));

                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .onTrue(new InstantCommand(
                                                () -> m_gyro.setYaw(0.0)));

                //new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                  //              .whileTrue(new AimAtTargetCommand(m_robotDrive, m_vision));
                new Trigger(() -> m_driverController.getRawAxis(Axis.kRightTrigger.value) > 0.1)
        .whileTrue(new AimAtTargetCommand(m_robotDrive, m_vision));
    
                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).
    onTrue(new InstantCommand( () -> System.out.println(m_gyro.getYaw())));

        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
              .onTrue(new AlignAprilTag(m_robotDrive, vision, m_robotDrive.getPose()));   
              new Trigger(() -> m_driverController.getRawAxis(Axis.kLeftTrigger.value) > 0.1).
              whileTrue(new RunCommand( () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                m_vision.keepPointedAtSpeaker(),
                fieldOrientedDrive, false), m_robotDrive)).
              onFalse(new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldOrientedDrive, false), m_robotDrive));
        }

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
                vision = new Vision(m_vision);
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
}
