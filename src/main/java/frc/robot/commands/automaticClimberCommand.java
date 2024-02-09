package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.intake.Intake;

public class automaticClimberCommand extends Command {

    private final Climber climber;

    public automaticClimberCommand(Climber climber, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber); 

    @Override
    public void execute() {

      if (XboxController.b.onTrue){
        m_climber.setLeftMotor(ClimberConstants.MAX_HEIGHT - 0.1);
        m_climber.setRightMotor(ClimberConstants.MAX_HEIGHT - 0.1);
      }
      else if (XboxController.a.onTrue){
        m_climber.setLeftMotor(ClimberConstants.MIN_HEIGHT + 0.1);
        m_climber.setRightMotor(ClimberConstants.MIN_HEIGHT + 0.1);
      }
    }
  }
}
