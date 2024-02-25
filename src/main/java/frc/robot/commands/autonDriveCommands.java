package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class autonDriveCommands extends Command {

    private final DriveSubsystem m_driveTrain;
    private final double distance;
    
 
	public autonDriveCommands(DriveSubsystem subsystem, double dist) {
        //initialize variables
        m_driveTrain = subsystem;
        distance = dist;
        addRequirements(m_driveTrain);

    }
    
    @Override
    public void initialize() { //FIX THIS CODE IT'S NOT CORRECT
        //strafe
        while (m_driveTrain.getPose().getX() != distance - 0.05) {
            m_driveTrain.drive(0, 0.1, 0, true);
        }
        m_driveTrain.drive(0,0,0,true);


          /* 
  private Command autonDriveStraight(double distance) {
    return new SequentialCommandGroup(
        new RunCommand(() -> m_robotDrive.drive(0, 0.1, 0, true), m_robotDrive),
        new WaitUntilCommand(() -> m_robotDrive.getPose().getY() == distance - 0.05), //add tolerance constant
        new RunCommand(() -> m_robotDrive.drive(0,0,0,true), m_robotDrive)
    );
}

private Command autonTurnInPlace(double angle) {
return new SequentialCommandGroup(
    new RunCommand(() -> m_robotDrive.drive(0, 0, 0.1, true), m_robotDrive),
    new WaitUntilCommand(() -> m_robotDrive.getRotation().getDegrees() == angle - 0.05), //add tolerance constant and ask about degree method
    new RunCommand(() -> m_robotDrive.drive(0,0,0,true), m_robotDrive)
);
}

private Command autonStrafeSideways(double distance) {
return new SequentialCommandGroup(
    new RunCommand(() -> m_robotDrive.drive(0.1, 0, 0, true), m_robotDrive),
    new WaitUntilCommand(() -> m_robotDrive.getPose().getX() == distance - 0.05), //add tolerance constant
    new RunCommand(() -> m_robotDrive.drive(0,0,0,true), m_robotDrive)
  );
}
*/
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // increased error tolerance so the command will finish in auton
        double butteryErrorTolerance = 0.05;
  
        // if (Math.abs(error) <= butteryErrorTolerance)
        // {
        //     return true;
        // }
        // return false;
        
        return false;
        
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    
        //m_driveTrain.setSpeed(0, 0, 0);

        DataLogManager.log("DriveForwardGivenDistance ended");
    }
}
