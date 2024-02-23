// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gyro;



public interface GyroIO {

  public static class GyroIOInputs {
    public double yawPositionRad = 0.0;

   
  }
  public double getYaw();
  public void setYaw(double yaw);
  public default void updateInputs(GyroIOInputs inputs) {}
}