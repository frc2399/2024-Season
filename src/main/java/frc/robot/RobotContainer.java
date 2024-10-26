// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO_Real;
import frc.robot.subsystems.drive.SwerveModuleIO_Sim;
import frc.robot.subsystems.drive.VisionIO;
import frc.robot.subsystems.drive.VisionIO_Hardware;
import frc.robot.subsystems.drive.VisionIO_Placebo;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        public DriveSubsystem robotDrive;
        private static GyroIO m_gyro;

        public boolean fieldOrientedDrive = true;
        public static boolean isInClimberMode = false;

        // swerve module IOs
        private SwerveModuleIO m_frontLeftIO;
        private SwerveModuleIO m_frontRightIO;
        private SwerveModuleIO m_rearLeftIO;
        private SwerveModuleIO m_rearRightIO;

        private VisionIO visionIO;

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
                setUpAuton();

        }

        public Command getAutonomousCommand() {
                return m_autoChooser.getSelected();
        }

        private void setUpSubsystems() {

                if (Robot.robotType == RobotType.SIMULATION) {
                        m_gyro = new GyroIOSim();
                        m_frontLeftIO = new SwerveModuleIO_Sim("front left");
                        m_frontRightIO = new SwerveModuleIO_Sim("front right");
                        m_rearLeftIO = new SwerveModuleIO_Sim("rear left");
                        m_rearRightIO = new SwerveModuleIO_Sim("rear right");

                        robotDrive = new DriveSubsystem(
                                        new SwerveModule(m_frontLeftIO),
                                        new SwerveModule(m_frontRightIO),
                                        new SwerveModule(m_rearLeftIO),
                                        new SwerveModule(m_rearRightIO), m_gyro, visionIO);

                } else {

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

                        m_gyro = new GyroIOPigeon2();
                        visionIO = new VisionIO_Hardware();

                        robotDrive = new DriveSubsystem(
                                        new SwerveModule(m_frontLeftIO),
                                        new SwerveModule(m_frontRightIO),
                                        new SwerveModule(m_rearLeftIO),
                                        new SwerveModule(m_rearRightIO), m_gyro, visionIO);

                }
        }

        // sets up auton commands
        private void setUpAuton() {
                m_autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autos/Selector", m_autoChooser);

        }

        // Configure default commands
        private void configureDefaultCommands() {
                // default command for drivetrain: drive based on controller inputs
                // actually driving robot
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> robotDrive.drive(
                                                                -(MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband)),
                                                                -(MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband)),
                                                                -(MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband)),
                                                                fieldOrientedDrive, false),
                                                robotDrive).withName("drive default"));
        }

        private void configureButtonBindingsDriver() {
                // while true with run commands
                m_driverController.x().whileTrue((new RunCommand(
                                () -> robotDrive.setX(),
                                robotDrive).withName("setx")));

                // driver b: reset gyro
                m_driverController.b().onTrue(new InstantCommand(() -> m_gyro.setYaw(0.0)));
                // driver a: align to speaker mode
                m_driverController.a().whileTrue(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> robotDrive.drive(
                                                                -(MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband)),
                                                                -(MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband)),
                                                                -(MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband)),
                                                                fieldOrientedDrive, true),
                                                robotDrive).withName("drive default"));
        }
}
