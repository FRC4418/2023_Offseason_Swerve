// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class swerveSubsystem extends SubsystemBase {
  /** Creates a new swerveSubsystem. */
  public swerveSubsystem() {
    final swerveModule fl = new swerveModule(
      Ports.motorPorts.frontLeftTurn, 
      Ports.motorPorts.frontLeftTurn, 
      false, 
      false, 
      Ports.encoderPorts.kFrontLeftDriveAbsoluteEncoderPort, 
      Constants.encoders.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
      false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
