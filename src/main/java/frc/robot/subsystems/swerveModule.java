// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class swerveModule extends SubsystemBase {
  /** Creates a new swerveModule. */
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  private final PIDController turningPidController;

  private final AnalogInput absoluteEncoder;

  private final boolean absoluteEncoderReversed;

  private final double absoluteEncoderOffsetRad;
  public swerveModule(
    int driveMotorID, 
    int turningMotorID, 
    boolean driveMotorReversed, 
    boolean turningMotorReversed,
    int absoluteEncoderID,
    double absoluteEncoderOffset,
    boolean absoluteEncoderReversed
  ) {
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderID);

    driveMotor = new WPI_TalonFX(driveMotorID);
    turningMotor = new WPI_TalonFX(turningMotorID);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    turningPidController = new PIDController(Constants.moduleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

public double getDrivePosition(){
  return driveMotor.getSelectedSensorPosition();
}
public double getTurningPosition(){
  return turningMotor.getSelectedSensorPosition();
}
public double getDriveVelocity(){
  return driveMotor.getSelectedSensorVelocity();
}
public double getTurningVelocity(){
  return turningMotor.getSelectedSensorVelocity();
}
public double getAbsoluteEncoder(){
  double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
  angle *= 2.0 * Math.PI;
  angle -= absoluteEncoderOffsetRad;
  return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}
public void resetEncoders(){
  driveMotor.setSelectedSensorPosition(0);
  turningMotor.setSelectedSensorPosition(getAbsoluteEncoder());
}
public SwerveModuleState getState(){
  return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}
public void setDesiredStates(SwerveModuleState state){
  if(Math.abs(state.speedMetersPerSecond) < 0.001){
    stop();
    return;
  }
  state = SwerveModuleState.optimize(state,getState().angle);
  driveMotor.set(state.speedMetersPerSecond / Constants.moduleConstants.kPhysicalMaxSpeedPerSeconds);
  turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "]", state.toString());
}
public void stop(){
  driveMotor.set(0);
  turningMotor.set(0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
