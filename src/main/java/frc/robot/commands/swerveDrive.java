// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import javax.xml.xpath.XPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.kinematics;
import frc.robot.subsystems.SwerveSubsystem;

public class swerveDrive extends CommandBase {
  /** Creates a new swerveDrive. */
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimit, yLimit, turningLimit;
  public swerveDrive(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.xLimit = new SlewRateLimiter(DriveConstants.kMaxAccelerationUnitsPerSecond);
    this.yLimit = new SlewRateLimiter(DriveConstants.kMaxAccelerationUnitsPerSecond);
    this.turningLimit = new SlewRateLimiter(DriveConstants.kMaxAngularAccelerationPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turning = turningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > Constants.controllerConstants.deadBand ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.controllerConstants.deadBand ? ySpeed : 0.0;
    turning = Math.abs(turning) > Constants.controllerConstants.deadBand ? turning : 0.0;

    xSpeed = xLimit.calculate(xSpeed) * DriveConstants.kMaxTeleOpSpeed;
    ySpeed = yLimit.calculate(ySpeed) * DriveConstants.kMaxTeleOpSpeed;
    turning = turningLimit.calculate(turning) * DriveConstants.kMaxTeleOpAngularSpeed;

    ChassisSpeeds chassisSpeeds;
    //ROBOT CENTRIC, FIELD RELATIVE CAN BE ADDED LATER
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turning);

    SwerveModuleState[] moduleStates = kinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
