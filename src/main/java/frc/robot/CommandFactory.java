package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CommandFactory {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;
    private final Arm arm;
    private final Climber climber;

    public CommandFactory(Shooter shooter, Indexer indexer, Intake intake, Arm arm, Climber climber) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.arm = arm;
        this.climber = climber;
    }

    public Command shootWhenUpToSpeed() {
        return Commands.parallel(
                shooter.setShootSpeed(arm.getSpeedFromArmHeight()),
                Commands.either(
                        Commands.parallel(indexer.runIndexer(IndexerConstants.INDEXER_IN_SPEED),
                                indexer.setIsIntooked(false)),
                        indexer.runIndexer(0),
                        shooter.isUpToSpeed(arm.getSpeedFromArmHeight())));
    }
}
