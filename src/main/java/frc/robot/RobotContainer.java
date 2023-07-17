// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.nio.file.FileSystem;

import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AutoGamepad cont = new AutoGamepad(0);
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "falcon"));
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    TeleopDrive closedTeleopDrive = new TeleopDrive(
      drivebase, 
      () -> MathUtil.applyDeadband(cont.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
      () -> MathUtil.applyDeadband(cont.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
      () -> cont.getRightY(), 
      () -> !cont.getDPadDown().getAsBoolean(), false, true);

    drivebase.setDefaultCommand(closedTeleopDrive);
  }


  private void configureBindings() {}


  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
  public void lockDrive(){
    drivebase.lock();
  }
}

