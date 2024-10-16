package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CommandFactory {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;
    private final Arm arm;

    public CommandFactory(Shooter shooter, Indexer indexer, Intake intake, Arm arm) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.arm = arm;
    }

    public Command shootWhenUpToSpeed() {
        return Commands.sequence(
                Commands.sequence(
                        shooter.setShootSpeed(arm.getSpeedFromArmHeight())),
                Commands.waitUntil(() -> shooter.isUpToSpeed(arm.getSpeedFromArmHeight())),
                indexer.runIndexer(IndexerConstants.INDEXER_IN_SPEED),
                Commands.sequence(
                        shooter.setShootSpeed(0),
                        intake.setMotor(0),
                        indexer.setIsIntooked(false)));

    }
}
