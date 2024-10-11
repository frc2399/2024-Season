package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CommandFactory {

    private final Shooter shooter;

    public CommandFactory(Shooter shooter) {
        this.shooter = shooter;
    }

}
