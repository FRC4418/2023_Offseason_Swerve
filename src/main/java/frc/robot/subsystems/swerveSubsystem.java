// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class swerveSubsystem extends SubsystemBase {
  /** Creates a new swerveSubsystem. */
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive drive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
  

    public swerveSubsystem() {

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
