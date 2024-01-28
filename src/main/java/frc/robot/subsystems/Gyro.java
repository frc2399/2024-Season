// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  static final Pigeon2 gyro = new Pigeon2(3, "rio");
  public static double yaw;
  public static double pitch;
  public static double roll;
  static double yawAccel;
  static double pitchAccel;
  static double rollAccel;
  /** Creates a new Gyro. */
  public Gyro() {
    gyro.setYaw(0);
    gyro.getYaw().setUpdateFrequency(100);
    gyro.getGravityVectorZ().setUpdateFrequency(100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    yaw = gyro.getYaw().getValue();
    pitch = gyro.getPitch().getValue();
    roll = gyro.getRoll().getValue();

    yawAccel = gyro.getAccelerationY().getValue();
    pitchAccel = gyro.getAccelerationX().getValue();
    rollAccel = gyro.getAccelerationZ().getValue();

    SmartDashboard.putNumber("Gyro/Yaw", yaw);
    SmartDashboard.putNumber("Gyro/Pitch", pitch);
    SmartDashboard.putNumber("Gyro/Roll", roll);
    SmartDashboard.putNumber("Gyro/Yaw Acceleration", yawAccel);
    SmartDashboard.putNumber("Gyro/Pitch Acceleration", pitchAccel);
    SmartDashboard.putNumber("Gyro/Roll Acceleration", rollAccel);
    SmartDashboard.putString("Gyro/Yaw Units", gyro.getYaw().getUnits()); /* These next 3 lines are just for testing, documentation for the pigeon is less than ideal, dunno what units it gives. The code is expecting degrees */
    SmartDashboard.putString("Gyro/Pitch Units", gyro.getPitch().getUnits());
    SmartDashboard.putString("Gyro/Roll Units", gyro.getRoll().getUnits());

    
  }

  public double getYaw() {
    return yaw;
  }

  public void resetYaw() {
    gyro.setYaw(0);
  }
}