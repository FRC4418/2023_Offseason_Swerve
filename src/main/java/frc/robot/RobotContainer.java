// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.doNothing;
import frc.robot.commands.swerveDrive;
import frc.robot.commands.zeroRobotHeading;
import frc.robot.subsystems.SwerveSubsystem;

import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  private final Field2d field = new Field2d();
  

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final AutoGamepad driver = new AutoGamepad(Ports.controllerPorts.driver);

  public RobotContainer() {
    SmartDashboard.putData("Field", field);
    field.setRobotPose(swerveSubsystem.m_Odometry.getPoseMeters());
    
    swerveSubsystem.setDefaultCommand(new swerveDrive(
      swerveSubsystem,
      () -> driver.getRightX(), 
      () -> driver.getRightY(), 
      () -> driver.getLeftY(), 
      () -> driver.getRawDPadUp()
      ));
    configureButtonBindings();
  }

  private void configureButtonBindings(){
    driver.getLeftButton().onTrue(new zeroRobotHeading(swerveSubsystem));
  }



  public Command getAutonomousCommand() {

    return new doNothing();
  }
}
