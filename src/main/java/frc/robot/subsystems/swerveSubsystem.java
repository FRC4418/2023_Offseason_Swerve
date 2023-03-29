// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class swerveSubsystem extends SubsystemBase {
  /** Creates a new swerveSubsystem. */
  private final swerveModule fl = new swerveModule(
    Ports.motorPorts.frontLeftDrive, 
    Ports.motorPorts.frontLeftTurn, 
    false, 
    false, 
    Ports.encoderPorts.kFrontLeftDriveAbsoluteEncoderPort, 
    Constants.encoders.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
    false);

  private final swerveModule fr = new swerveModule(
    Ports.motorPorts.frontRightDrive, 
    Ports.motorPorts.frontRightTurn, 
    false, 
    false, 
    Ports.encoderPorts.kFrontRightDriveAbsoluteEncoderPort, 
    Constants.encoders.kFrontRightDriveAbsoluteEncoderOffsetRad, 
    false);

  private final swerveModule bl = new swerveModule(
    Ports.motorPorts.backLeftDrive, 
    Ports.motorPorts.backLeftTurn,
    true, 
    false, 
    Ports.encoderPorts.kBackLeftDriveAbsoluteEncoderPort, 
    Constants.encoders.kBackLeftDriveAbsoluteEncoderOffsetRad,
    false);

  private final swerveModule br = new swerveModule(
    Ports.motorPorts.backRightDrive,
    Ports.motorPorts.backRightDrive, 
    true, 
    false, 
    Ports.encoderPorts.kBackRightDriveAbsoluteEncoderPort, 
    Constants.encoders.kBackRightDriveAbsoluteEncoderOffsetRad, 
    false);

  private final AHRS gyro = new AHRS();
  
  public swerveSubsystem() {
    new Thread(() ->  {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}
    }).start();
  }
  public void zeroHeading(){
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
