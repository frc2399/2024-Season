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
import frc.robot.subsystems.shooter.ShooterHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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
        private DriveSubsystem robotDrive;
        private static GyroIO gyro;

        public boolean fieldOrientedDrive = true;
        public static boolean isInClimberMode = false;

        // swerve module IOs
        private SwerveModuleIO frontLeftIO;
        private SwerveModuleIO frontRightIO;
        private SwerveModuleIO rearLeftIO;
        private SwerveModuleIO rearRightIO;

        // subsystems
        public static ShooterSubsystem shooter;
        public static Intake intake;
        public static Indexer indexer;
        public static Climber climber;
        public static Arm arm;
        public static Vision vision;
        public static LED Led;

        // subsystem IOs
        ShooterIO shooterIO;
        IntakeIO intakeIO;
        IndexerIO indexerIO;
        ArmIO armIO;
        ClimberIO climberIO;
        VisionIO visionIO;

        // auton chooser
        private static SendableChooser<Command> autoChooser;
        ComplexWidget autonChooserWidget;

        // The driver and operator controllers
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

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
                return autoChooser.getSelected();
        }

        private void setUpSubsystems() {

                if (Robot.robotType == RobotType.SIMULATION) {
                        indexerIO = new SimIndexer();
                        shooterIO = new SimShooter();
                        intakeIO = new SimIntake();
                        climberIO = new ClimberSim();
                        armIO = new SimArm();
                        gyro = new GyroIOSim();
                        frontLeftIO = new SwerveModuleIO_Sim("front left");
                        frontRightIO = new SwerveModuleIO_Sim("front right");
                        rearLeftIO = new SwerveModuleIO_Sim("rear left");
                        rearRightIO = new SwerveModuleIO_Sim("rear right");

                        robotDrive = new DriveSubsystem(
                                        new SwerveModule(frontLeftIO),
                                        new SwerveModule(frontRightIO),
                                        new SwerveModule(rearLeftIO),
                                        new SwerveModule(rearRightIO), gyro);
                        visionIO = new VisionSim(robotDrive);

                } else {

                        frontLeftIO = new SwerveModuleIO_Real(DriveConstants.kFrontLeftDrivingCanId,
                                        DriveConstants.kFrontLeftTurningCanId,
                                        DriveConstants.kFrontLeftChassisAngularOffset,
                                        "front left");
                        frontRightIO = new SwerveModuleIO_Real(DriveConstants.kFrontRightDrivingCanId,
                                        DriveConstants.kFrontRightTurningCanId,
                                        DriveConstants.kFrontRightChassisAngularOffset,
                                        "front right");
                        rearLeftIO = new SwerveModuleIO_Real(DriveConstants.kRearLeftDrivingCanId,
                                        DriveConstants.kRearLeftTurningCanId,
                                        DriveConstants.kRearLeftChassisAngularOffset,
                                        "rear left");
                        rearRightIO = new SwerveModuleIO_Real(DriveConstants.kRearRightDrivingCanId,
                                        DriveConstants.kRearRightTurningCanId,
                                        DriveConstants.kRearRightChassisAngularOffset,
                                        "rear right");

                        indexerIO = new RealIndexer();
                        shooterIO = new ShooterHardware();
                        intakeIO = new RealIntake();
                        climberIO = new ClimberReal();
                        armIO = new RealArm();
                        gyro = new GyroIOPigeon2();
                        visionIO = new VisionReal();

                        robotDrive = new DriveSubsystem(
                                        new SwerveModule(frontLeftIO),
                                        new SwerveModule(frontRightIO),
                                        new SwerveModule(rearLeftIO),
                                        new SwerveModule(rearRightIO), gyro);

                }

                climber = new Climber(climberIO);
                arm = new Arm(armIO);
                shooter = new ShooterSubsystem(shooterIO);
                indexer = new Indexer(indexerIO);
                intake = new Intake(intakeIO);
                vision = new Vision(visionIO);
                Led = new LED(vision, indexer);

        }

        // sets up auton commands
        private void setUpAuton() {
                NamedCommands.registerCommand("intake", intakeWithHeightRestriction());
                NamedCommands.registerCommand("Intake", intakeWithHeightRestriction());
                NamedCommands.registerCommand("intake for time", intakeForTime(intake, indexer));
                NamedCommands.registerCommand("SHORT intake for time", shortIntakeForTime(intake, indexer));
                NamedCommands.registerCommand("AimToTarget", Commands.print("aimed to target!"));
                NamedCommands.registerCommand("SetArmPosition", makeSetPositionCommandAuton(arm, 0.74));
                NamedCommands.registerCommand("Set Arm Wingleft", makeSetPositionCommandAuton(arm, 0.785));
                NamedCommands.registerCommand("SetArmDown", makeSetPositionCommandAuton(arm, 0.335));
                NamedCommands.registerCommand("AutoShoot", outtakeAndShootAfterDelay());
                NamedCommands.registerCommand("intake and outtake", intakeAndOuttake());
                NamedCommands.registerCommand("outtake", outtake());

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autos/Selector", autoChooser);

        }

        // Configure default commands
        private void configureDefaultCommands() {
                // default command for the shooter: do nothing
                shooter.setDefaultCommand(
                                new RunCommand(
                                                () -> shooter.setMotor(0),
                                                shooter).withName("drive default"));

                // default command for intake: do nothing
                intake.setDefaultCommand(
                                new RunCommand(
                                                () -> intake.setMotor(0),
                                                intake).withName("drive default"));

                // default command for indexer: do nothing
                indexer.setDefaultCommand(
                                new RunCommand(
                                                () -> indexer.setMotor(0),
                                                indexer).withName("drive default"));

                // default command for climber: do nothing
                climber.setDefaultCommand(
                                new RunCommand(
                                                () -> climber.setMotors(0),
                                                climber));

                // Arm default command; do nothing but with gravity compensation so it stays
                // where it is.
                // Setpoint is in RADIANS
                arm.setEncoderPosition(arm.getAbsoluteEncoderPosition());
                arm.setDefaultCommand(
                                new RunCommand(() -> arm.setSpeedGravityCompensation(0), arm)
                                                .withName("drive default"));

                // default command for drivetrain: drive based on controller inputs
                // actually driving robot
                robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> robotDrive.drive(
                                                                -(MathUtil.applyDeadband(
                                                                                driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband)),
                                                                -(MathUtil.applyDeadband(
                                                                                driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband)),
                                                                -(MathUtil.applyDeadband(
                                                                                driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband)),
                                                                fieldOrientedDrive),
                                                robotDrive).withName("drive default"));

        }

        private void configureButtonBindingsDriver() {
                // while true with run commands
                driverController.y()
                                .onTrue(new InstantCommand(
                                                () -> arm.setEncoderPosition(arm.getAbsoluteEncoderPosition())));
                driverController.x().whileTrue((new RunCommand(
                                () -> robotDrive.setX(),
                                robotDrive).withName("setx")));

                // driver left bumper: manual shoot
                // gets arm height to assign to speed. lower arm, means cloesr to speaekr, so
                // shoots less forecfully
                driverController.leftBumper().whileTrue(new ParallelCommandGroup(
                                new RunCommand(() -> shooter.setMotor(arm.getSpeedFromArmHeight()), shooter),
                                new RunCommand(() -> indexer.setIsIntooked(false))));

                // driver right bumper: auto-shoot
                driverController.rightBumper().onTrue(shootWhenUpToSpeed());

                // driver right trigger: manual intake with arm height restriction
                // only intakes if arm is lowered
                driverController.rightTrigger().whileTrue(new automaticIntakeAndIndexer(indexer, intake, arm));

                // driver left trigger: outtake
                driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
                                new RunCommand(() -> intake.setMotor(-0.3), intake),
                                new RunCommand(() -> indexer.setMotor(-0.3), indexer),
                                new RunCommand(() -> indexer.setIsIntooked(false))));

                // driver b: reset gyro
                driverController.b().onTrue(new InstantCommand(() -> gyro.setYaw(0.0)));
                // driver a: align to speaker mode
                driverController.a().whileTrue(
                                new RunCommand(() -> robotDrive.drive(
                                                -Math.pow(MathUtil.applyDeadband(driverController.getLeftY(),
                                                                OIConstants.kDriveDeadband), 3),
                                                -Math.pow(MathUtil.applyDeadband(driverController.getLeftX(),
                                                                OIConstants.kDriveDeadband), 3),
                                                vision.keepPointedAtSpeaker(),
                                                fieldOrientedDrive), robotDrive))
                                .onFalse(new InstantCommand(() -> vision.makeDriveTrainAlignedFalse()));

        }

        private void configureButtonBindingsOperatorClimber() {
                // operater left trigger: climber mode: left climber up
                operatorController.leftTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setLeftSpeed(0.5), climber));

                // operater right trigger: climber mode: right climber up
                operatorController.rightTrigger().and(() -> isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setRightSpeed(0.5), climber));

                // operater left bumper: climber mode: left climber down
                operatorController.leftBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setLeftSpeed(-0.5), climber));

                // operater right bumper: climber mode: right climber down
                operatorController.rightBumper().and(() -> isInClimberMode).whileTrue(new RunCommand(
                                () -> climber.setRightSpeed(-0.5), climber));

                // operator b (climber mode): automatic climber up
                operatorController.b().and(() -> isInClimberMode)
                                .whileTrue(new RunCommand(() -> climber.setMotors(0.9), climber));

                // operator a (climber mode): automatic climber down
                operatorController.a().and(() -> isInClimberMode)
                                .whileTrue(new RunCommand(() -> climber.setMotors(-0.9), climber));

                // operator x: switch operator controller modes
                operatorController.x().onTrue(new InstantCommand(() -> isInClimberMode = !isInClimberMode));

        }

        private void configureButtonBindingsOperatorNotClimber() {

                operatorController.leftTrigger().and(() -> !isInClimberMode).whileTrue(
                                makeSetPositionCommandVision(arm))
                                .onFalse(new InstantCommand(() -> vision.makeArmAlignedFalse()));

                operatorController.rightTrigger().and(() -> !isInClimberMode).whileTrue(
                                new ParallelCommandGroup(
                                                new RunCommand(() -> intake.setMotor(1), intake),
                                                new RunCommand(() -> indexer.setMotor(1), indexer)));

                // operater a: arm to intake/subwoofer angle
                operatorController.a().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(arm, 0.31));

                // operator b: arm to podium shot angle
                operatorController.b().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(arm, 0.66));

                // operator y: arm to amp angle
                operatorController.y().and(() -> !isInClimberMode).onTrue(makeSetPositionCommand(arm, 1.58));

                // operator right bumper: intake
                operatorController.rightBumper().and(() -> !isInClimberMode)
                                .whileTrue(new RunCommand(() -> indexer.setMotor(0.3), indexer));

                // operator left bumper: outtake
                // outtake a little bittt to get shooter up to speed
                operatorController.leftBumper().and(() -> !isInClimberMode)
                                .onTrue(new SequentialCommandGroup(
                                                new RunCommand(() -> indexer.setMotor(-0.3), indexer)
                                                                .withTimeout(0.1),
                                                new InstantCommand(() -> indexer.setIsIntooked(false))));

                // TODO test this!
                operatorController.axisGreaterThan(5, 0.1)
                                .whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.1));
                operatorController.axisLessThan(5, -0.1)
                                .whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.1));
        }

        public static Command makeSetPositionCommand(Arm arm,
                        double target) {
                return new SequentialCommandGroup(
                                new ConditionalCommand(new InstantCommand(() -> {
                                }), new InstantCommand(() -> arm.enable(), arm), () -> arm.isEnabled()),
                                new RunCommand(() -> arm.setGoal(target), arm));
        }

        private Command makeSetPositionCommandVision(Arm arm) {
                DoubleSupplier target = () -> (vision.keepArmAtAngle((arm.getAbsoluteEncoderPosition())));
                return new SequentialCommandGroup(
                                new ConditionalCommand(new InstantCommand(() -> {
                                }), new InstantCommand(() -> arm.enable(), arm), () -> arm.isEnabled()),
                                new RunCommand(() -> arm.setGoal(target.getAsDouble()), arm));
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
                                                new RunCommand(() -> indexer.setMotor(0.8), indexer)).withTimeout(0.28),
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> intake.setMotor(.0), intake),
                                                new InstantCommand(() -> indexer.setMotor(0.0), indexer)));

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
                                new RunCommand(() -> intake.setMotor(speed), intake),
                                new RunCommand(() -> indexer.setMotor(speed), indexer)).withTimeout(0.4);
        }

        // waiting 0.5 seconds to get shooter up to speed
        private Command shootWhenUpToSpeed() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WaitUntilCommand(() -> shooter
                                                                                .getEncoderSpeed() >= (arm
                                                                                                .getSpeedFromArmHeight()
                                                                                                * Constants.ShooterConstants.SHOOT_MAX_SPEED_RPS)),
                                                                new RunCommand(() -> indexer.setMotor(
                                                                                Constants.IndexerConstants.INDEXER_IN_SPEED),
                                                                                indexer)),
                                                new RunCommand(() -> shooter.setMotor(arm.getSpeedFromArmHeight()),
                                                                shooter))
                                                .withTimeout(1), // 0.75
                                new InstantCommand(() -> shooter.setMotor(0), shooter),
                                new InstantCommand(() -> indexer.setMotor(0), indexer),
                                new InstantCommand(() -> indexer.setIsIntooked(false)));

        }

        private Command outtakeAndShootAfterDelay() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.25),
                                                                new RunCommand(() -> indexer.setMotor(
                                                                                Constants.IndexerConstants.INDEXER_IN_SPEED),
                                                                                indexer),
                                                                new RunCommand(() -> indexer.setIsIntooked(false))),
                                                new RunCommand(() -> shooter.setMotor(arm.getSpeedFromArmHeight()),
                                                                shooter))
                                                .withTimeout(0.5),
                                new InstantCommand(() -> shooter.setMotor(0), shooter),
                                new InstantCommand(() -> indexer.setMotor(0), indexer),
                                new InstantCommand(() -> indexer.setIsIntooked(false)));
        }

        private Command intakeWithHeightRestriction() {
                return new ConditionalCommand(
                                new ParallelCommandGroup(
                                                new RunCommand(() -> intake.setMotor(1), intake),
                                                new RunCommand(() -> indexer.setMotor(1), indexer)),
                                new ParallelCommandGroup(
                                                new RunCommand(() -> intake.setMotor(0), intake),
                                                new RunCommand(() -> indexer.setMotor(0), indexer)),
                                () -> arm.getEncoderPosition() < 0.5 && indexer.isIntooked == false);
        }

        private Command intakeAndOuttake() {
                return new SequentialCommandGroup(
                                new RunCommand(() -> indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED),
                                                indexer)
                                                .withTimeout(0.2),
                                new RunCommand(() -> indexer.setMotor(-0.05), indexer).withTimeout(0.1));
        }

        private Command outtake() {
                return new SequentialCommandGroup(
                                new RunCommand(() -> indexer.setMotor(-0.15), indexer)
                                                .withTimeout(0.3),
                                new RunCommand(() -> indexer.setMotor(0), indexer).withTimeout(0.1));
        }
}
