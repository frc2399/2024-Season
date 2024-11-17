package frc.robot;

import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardware;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOSim;

public class SubsystemFactory {

    private boolean isSimulation;

    public DriveSubsystem buildDriveSubsystem(boolean isSimulation) {

        this.isSimulation = isSimulation;

        if (isSimulation) {
            return new DriveSubsystem(new SwerveModuleSim("Front Left"), new SwerveModuleSim("Front Right"),
                    new SwerveModuleSim("Rear Left"), new SwerveModuleSim("Rear Right"), new GyroIOSim());
        }

        else {
            SwerveModule frontLeft = new SwerveModuleHardware(DriveSubsystem.FRONT_LEFT_DRIVING_CAN_ID,
                    DriveSubsystem.FRONT_LEFT_TURNING_CAN_ID, DriveSubsystem.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
                    "Front Left")
            return new DriveSubsystem(null, null, null, null, null);
        }
    }
}
