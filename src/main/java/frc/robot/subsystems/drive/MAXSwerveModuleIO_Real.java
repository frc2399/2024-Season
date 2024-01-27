package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;

public class MAXSwerveModuleIO_Real {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
}
