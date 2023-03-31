// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;

public class swerveSubsystem extends SubsystemBase {
  /** Creates a new swerveSubsystem. */
  private final SwerveDrive swerveDrive;

  private SwerveAutoBuilder autoBuilder = null;
  
    public swerveSubsystem(File directory) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      try{
        swerveDrive = new SwerveParser(directory).createSwerveDrive();
      } catch (IOException e){
        System.out.println("The swerve could not be created" + e.getStackTrace());
        throw new RuntimeException(e);
      }
    }
public void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop){
  swerveDrive.drive(translation, rotation, isFieldRelative, isOpenLoop);
}
public SwerveKinematics2 getKinematics(){
  return swerveDrive.kinematics;
}
public Pose2d getPose(){
  return swerveDrive.getPose();
}
public void poseTrajectory(Trajectory traj){
  swerveDrive.postTrajectory(traj);
} 
public void zeroGyro()
{
  swerveDrive.zeroGyro();
}
public void setMotorBrake(boolean brake)
{
  swerveDrive.setMotorIdleMode(brake);
}
public Rotation2d getHeading()
{
  return swerveDrive.getYaw();
}
public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
{
  xInput = Math.pow(xInput, 3);
  yInput = Math.pow(yInput, 3);
  return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
}
public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
{
  xInput = Math.pow(xInput, 3);
  yInput = Math.pow(yInput, 3);
  return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
}
public ChassisSpeeds getFieldVelocity()
{
  return swerveDrive.getFieldVelocity();
}
public ChassisSpeeds getRobotVelocity()
{
  return swerveDrive.getRobotVelocity();
}
public SwerveController getSwerveController()
{
  return swerveDrive.swerveController;
}
public SwerveDriveConfiguration getSwerveDriveConfiguration()
{
  return swerveDrive.swerveDriveConfiguration;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
  }
}
