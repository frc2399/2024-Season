package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CommandFactory {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;

    public CommandFactory(Shooter shooter, Indexer indexer, Intake intake) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
    }

    public Command shootWhenUpToSpeed(double speed) {
        return Commands.parallel(shooter.setShootSpeed(speed),
                Commands.sequence(Commands.waitUntil(() -> shooter.isUpToSpeed(speed)),
                        indexer.runIndexer(IndexerConstants.INDEXER_IN_SPEED)));
    }

    /**
     * return new SequentialCommandGroup(
     * new ParallelCommandGroup(
     * new SequentialCommandGroup(
     * new WaitUntilCommand(() -> m_shooter
     * .getEncoderSpeed() >= (m_arm.getSpeedFromArmHeight() *
     * Constants.ShooterConstants.SHOOT_MAX_SPEED_RPS)),
     * new RunCommand(() ->
     * m_indexer.setMotor(Constants.IndexerConstants.INDEXER_IN_SPEED), m_indexer)),
     * new RunCommand(() -> m_shooter.setMotor(m_arm.getSpeedFromArmHeight()),
     * m_shooter)).withTimeout(1), // 0.75
     * new InstantCommand(() -> m_shooter.setMotor(0), m_shooter),
     * new InstantCommand(() -> m_indexer.setMotor(0), m_indexer),
     * new InstantCommand(() -> m_indexer.setIsIntooked(false)));
     */
}
