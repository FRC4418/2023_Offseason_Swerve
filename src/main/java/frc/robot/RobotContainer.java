// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.doNothing;
import frc.robot.subsystems.swerveSubsystem;

import java.io.File;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final Field2d field = new Field2d();

  private final AutoGamepad driver = new AutoGamepad(Ports.controllerPorts.driver);

  private PathPlannerTrajectory traj = PathPlanner.loadPath("Test1", new PathConstraints(1.5, 1.5));

  private final swerveSubsystem driveBase =  new swerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));

  public RobotContainer() {
    PathPlannerServer.startServer(5811);
    SmartDashboard.putData("Field", field);
    
    // Configure the trigger bindings
    configureButtonBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(driveBase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(driver.getLeftY(),
                                                                                       0.01),
                                                          () -> MathUtil.applyDeadband(driver.getLeftX(),
                                                                                       0.01),
                                                          () -> -driver.getRightX(),
                                                          () -> -driver.getRightY(),
                                                          false);

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(driveBase,
                                                                         () ->
                                                                             MathUtil.applyDeadband(driver.getLeftY(),
                                                                                                    0.01),
                                                                         () -> MathUtil.applyDeadband(driver.getLeftX(),
                                                                                                      0.01),
                                                                         () -> driver.getLeftX(), false);
    TeleopDrive simClosedFieldRel = new TeleopDrive(driveBase,
                                                    () -> MathUtil.applyDeadband(driver.getLeftY(),
                                                                                 0.01),
                                                    () -> MathUtil.applyDeadband(driver.getLeftX(),
                                                                                 0.01),
                                                    () -> driver.getRightX(), () -> true, false, true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        driveBase,
        () -> MathUtil.applyDeadband(driver.getLeftY(), 0.01),
        () -> MathUtil.applyDeadband(driver.getLeftX(), 0.01),
        () -> -driver.getRightX(), () -> true, false, true);

    driveBase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
  }

  private void configureButtonBindings(){
    
  }



  public Command getAutonomousCommand() {

    return new doNothing();
  }
}
