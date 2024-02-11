package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class automaticClimberCommand extends Command {

  private final Climber climber;
  private double setpoint;

  public automaticClimberCommand(Climber climber, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.setpoint = setpoint;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    while (climber.getLeftEncoderPosition() != setpoint || climber.getRightEncoderPosition() != setpoint) {
      if (climber.getLeftEncoderPosition() != setpoint) {
        if (climber.getLeftEncoderPosition() < setpoint) {
          climber.setLeftSpeed(0.2);

        } else if (climber.getLeftEncoderPosition() > setpoint) {
          climber.setLeftSpeed(0.2);
        }
      }

      if (climber.getRightEncoderPosition() != setpoint) {
        if (climber.getRightEncoderPosition() < setpoint) {
          climber.setRightSpeed(0.2);
        } else if (climber.getLeftEncoderPosition() > setpoint) {
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
}
