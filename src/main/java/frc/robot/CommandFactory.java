package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class CommandFactory {

    private final ShooterSubsystem shooter;

    public CommandFactory(ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;

    }

}
