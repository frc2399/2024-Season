package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO climberIO;
    private Debouncer leftDebouncer, rightDebouncer;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
        leftDebouncer = new Debouncer(0.025);
        rightDebouncer = new Debouncer(0.025);

    }

    public Command runClimber() {

        return this.run(() -> climberIO.setLeftSpeed(1.0));

    }

    public boolean isLeftSideStalling() {
        return leftDebouncer
                .calculate(Math.abs(climberIO.getLeftCurrent()) > Constants.ClimberConstants.CURRENT_THRESHOLD);
    }

    public boolean isRightSideStalling() {
        return rightDebouncer
                .calculate(Math.abs(climberIO.getRightCurrent()) > Constants.ClimberConstants.CURRENT_THRESHOLD);
    }

}