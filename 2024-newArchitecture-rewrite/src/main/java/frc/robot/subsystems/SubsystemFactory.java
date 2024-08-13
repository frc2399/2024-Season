package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SubsystemFactory {
  
  private DriveSubsystem RobotDrive;


  public DriveSubsystem getDrivetrain() {
    if (Robot.robotType == RobotType.SIMULATION) {
      RobotDrive = new DriveSubsystem();
    } else {
      RobotDrive = new DriveSubsystem();
    }
    return RobotDrive;
  }
}
