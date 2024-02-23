package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class automaticClimberCommand extends Command {

  private final Climber climber;
  private double setpoint;

  public automaticClimberCommand(Climber climber, double setpoint) {
    this.climber = climber;
    this.setpoint = setpoint;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    while (shouldRunWhileLoop(setpoint, climber)) {
      if (climber.getLeftEncoderPosition() < (setpoint - ClimberConstants.SETPOINT_RANGE)
          || (climber.getLeftEncoderPosition() > (setpoint - ClimberConstants.SETPOINT_RANGE))) {
        if (climber.getLeftEncoderPosition() < (setpoint - ClimberConstants.SETPOINT_RANGE)
            || climber.getLeftEncoderPosition() < setpoint) {
          climber.setLeftSpeed(0.2);

        } else if (climber.getLeftEncoderPosition() > (setpoint + ClimberConstants.SETPOINT_RANGE)
            || climber.getLeftEncoderPosition() > setpoint) {
          climber.setLeftSpeed(-0.2);
        }
      }

      if (climber.getRightEncoderPosition() < (setpoint - ClimberConstants.SETPOINT_RANGE)
          || (climber.getLeftEncoderPosition() > (setpoint - ClimberConstants.SETPOINT_RANGE))) {
        if (climber.getRightEncoderPosition() < (setpoint - ClimberConstants.SETPOINT_RANGE)
            || climber.getRightEncoderPosition() < setpoint) {
          climber.setRightSpeed(0.2);
        } else if (climber.getRightEncoderPosition() > (setpoint + ClimberConstants.SETPOINT_RANGE)
            || climber.getRightEncoderPosition() > setpoint) {
          climber.setRightSpeed(-0.2);
        }
      }
    }
    climber.setLeftSpeed(0.0);
    climber.setRightSpeed(0.0);
  }

  @Override
  public void end(boolean interrupted) {

    climber.setLeftSpeed(0.0);
    climber.setRightSpeed(0.0);
  }

  public static boolean shouldRunWhileLoop(double setpoint, Climber climber) {
    boolean isRightAtPosition = (setpoint - ClimberConstants.SETPOINT_RANGE) < climber.getRightEncoderPosition()
        && (setpoint + ClimberConstants.SETPOINT_RANGE) > climber.getRightEncoderPosition();

    boolean isLeftAtPosition = (setpoint - ClimberConstants.SETPOINT_RANGE) < climber.getLeftEncoderPosition()
        && (setpoint + ClimberConstants.SETPOINT_RANGE) > climber.getLeftEncoderPosition();

    return !isRightAtPosition || !isLeftAtPosition;

  }
}
