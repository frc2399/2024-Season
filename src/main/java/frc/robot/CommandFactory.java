package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class CommandFactory {

    private final ShooterSubsystem shooter;
    private final Indexer indexer;
    private final Intake intake;
    private final Arm arm;
    private final Climber climber;

    public CommandFactory(ShooterSubsystem shooter, Indexer indexer, Intake intake, Arm arm, Climber climber) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.arm = arm;
        this.climber = climber;
    }

    public Command getSpeedFromArmHeightCommand() {
        return new RunCommand(() -> shooter.setMotor(arm.getSpeedFromArmHeight()), shooter);
    }

}