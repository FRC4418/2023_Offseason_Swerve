// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.kinematics;
import frc.robot.commands.swerveDrive;

public class SwerveSubsystem extends SubsystemBase {
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

  public SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
    kinematics.kDriveKinematics,
    gyro.getRotation2d(),
    new SwerveModulePosition[]{
      fl.getPosition(),
      fr.getPosition(),
      br.getPosition(),
      bl.getPosition()}
      );

  public SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
    kinematics.kDriveKinematics, getRotation2d(), 
    new SwerveModulePosition[]{
      fr.getPosition(),
      fl.getPosition(),
      br.getPosition(),
      bl.getPosition()
    }, 
    new Pose2d());

  
  
  public SwerveSubsystem() {
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
  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public Rotation2d getRotation2d(){
    return gyro.getRotation2d();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    estimator.update(
      getRotation2d(),
      new SwerveModulePosition[]{
        fr.getPosition(), 
        fl.getPosition(), 
        br.getPosition(), 
        bl.getPosition()
      });
    
    m_Odometry.update(
      gyro.getRotation2d(), 
      new SwerveModulePosition[]{
        fr.getPosition(), 
        fl.getPosition(), 
        br.getPosition(), 
        bl.getPosition()
    });
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Current Pose", m_Odometry.getPoseMeters().toString());
  }
  public void stopModules(){
    fl.stop();
    fr.stop();
    bl.stop();
    br.stop();
  }
  public ChassisSpeeds getChassisSpeeds(){
    var frontLeftState = fl.getState();
    var frontRightState = fr.getState();
    var backLeftState = bl.getState();
    var backRightState = br.getState();
    ChassisSpeeds chassisSpeeds = kinematics.kDriveKinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState, backRightState);
    return chassisSpeeds;
  }
  public void setModuleStates(SwerveModuleState[] desiredStates){
    //ALL VALUES FOR DESATURATION WILL NEED TO BE UPDATED
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, 
      getChassisSpeeds(), 
      Constants.moduleConstants.kPhysicalMaxSpeedPerSeconds, 
      Constants.moduleConstants.kPhysicalMaxSpeedPerSeconds, 
      Constants.moduleConstants.kPhysicalMaxSpeedPerSeconds);
    fr.setDesiredStates(desiredStates[0]);
    fl.setDesiredStates(desiredStates[1]);
    br.setDesiredStates(desiredStates[2]);
    bl.setDesiredStates(desiredStates[3]);
  }
  public void resetOdometry(Pose2d pose){
    m_Odometry.resetPosition(
      pose.getRotation(), 
      new SwerveModulePosition[]{
        fr.getPosition(), 
        fl.getPosition(), 
        br.getPosition(), 
        bl.getPosition()
      }, pose);
  }
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }
  public Command drivePath(PathPlannerTrajectory traj, boolean isFirstPath){
    PathPlannerServer.sendActivePath(traj.getStates());
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if(isFirstPath){
          this.resetOdometry(traj.getInitialHolonomicPose());
        }
      }, this),
      new PPSwerveControllerCommand(
        traj,
        this::getPose,
        kinematics.kDriveKinematics,
        new PIDController(0.2,0,0),
        new PIDController(0.2,0,0),
        new PIDController(0.2,0,0),
        this::setModuleStates,
        false,
        this
    )
    );
  }
}
