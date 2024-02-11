package frc.robot.commands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;

public class automaticIntakeAndIndexer extends Command {
    private final Indexer indexer;
    private final Intake intake;
    public static boolean isIntooked = false;
    public boolean pieceIntooked = false;

    public automaticIntakeAndIndexer(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        addRequirements(indexer);
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        while (isIntooked == false) {
            if (indexer.getIsBeamBroken()) {
                isIntooked = true;
                indexer.setIsIntooked(true);
            }
            intake.setMotor(5);
            indexer.setMotor(5);
        }
    }
}
