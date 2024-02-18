package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class automaticIntakeAndIndexer extends Command {
    private final Indexer indexer;
    private final Intake intake;
    private Timer timer;

    public automaticIntakeAndIndexer(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        addRequirements(indexer);
        this.intake = intake;
        addRequirements(intake);
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void execute() {
        if (indexer.isStalling()) {
            timer.start();
            if(timer.get() > 0.1) {
                indexer.setIsIntooked(true);
                indexer.setMotor(0);
                intake.setMotor(0); }
        } else {
            intake.setMotor(.3);
            indexer.setMotor(.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setMotor(0);
        intake.setMotor(0);
    }
}
