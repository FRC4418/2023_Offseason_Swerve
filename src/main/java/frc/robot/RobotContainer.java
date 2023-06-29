// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "falcon"));

  private AutoGamepad driver = new AutoGamepad(0);


  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AbsoluteFieldDrive fieldRelativeDrive = new AbsoluteFieldDrive(
      drivebase, 
      // () ->  MathUtil.applyDeadband(Constants.OperatorConstants.LEFT_X_DEADBAND, driver.getLeftY()),
      // () -> MathUtil.applyDeadband(Constants.OperatorConstants.LEFT_Y_DEADBAND, driver.getLeftX()),  
      // () -> MathUtil.applyDeadband(Constants.OperatorConstants.Rot_DEADBAND, driver.getRightX()), 
      () -> driver.getLeftY(),
      () -> driver.getLeftX(),
      () ->  driver.getRightX(),
      // () -> MathUtil.applyDeadband(Constants.OperatorConstants.Rot_DEADBAND, driver.getRightX()),
      false);
    drivebase.setDefaultCommand(fieldRelativeDrive);
  }

  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command drivePath(String nameOfPath, boolean isFirstPath) {
    PathPlannerTrajectory drivePath1 = PathPlanner.loadPath(nameOfPath, new PathConstraints(0.5, 3.0));
    PathPlannerServer.sendActivePath(drivePath1.getStates());
    // An example command will be run in autonomous
    return new SequentialCommandGroup((
      new InstantCommand(() -> {
        if(isFirstPath){
          drivebase.resetOdometry(drivePath1.getInitialHolonomicPose());
        }
      })
    ),
    new PPSwerveControllerCommand(
      drivePath1,
      drivebase::getPose , 
      new PIDController(0.1, 0, 0), 
      new PIDController(0.1, 0, 0), 
      new PIDController(0.1, 0, 0), 
      drivebase::setChassisSpeeds, 
      drivebase) 
    );
  }

  public Command getAutonomousCommand(){
    return drivePath("Path1", true);
  }
  
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
