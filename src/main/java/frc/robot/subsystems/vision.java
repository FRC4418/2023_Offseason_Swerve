// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new vision. */
  private static PhotonCamera camera;
  private PhotonPipelineResult frame;
  private PhotonTrackedTarget target;
  private Transform3d cameraToDrivetrain;
  private Pose2d currentPose;
  private SwerveSubsystem driveBase;
  private Transform3d transform2Target3D;
  private Transform2d transform2Target2D;
  public Vision(SwerveSubsystem driveBase) {
    this.driveBase = driveBase;
    camera = new PhotonCamera("USB_Camera");
  }

  public PathPlannerTrajectory makePath2Target(){
    //Target acqusition/verify may need to change
    transform2Target3D = target.getBestCameraToTarget();
    
    double xTrans = transform2Target3D.getX();
    double yTrans = transform2Target3D.getY();

    Rotation3d rot = transform2Target3D.getRotation();
    double rotAngle = rot.getAngle();

    currentPose = driveBase.getPose();
    double currentXPos = currentPose.getX();
    double currentYPos = currentPose.getX();
    Rotation2d currentRotation = currentPose.getRotation();

    PathPlannerTrajectory path2Target = PathPlanner.generatePath(
      new PathConstraints(0.75, 0.5), 
      new PathPoint(new Translation2d(currentXPos, currentYPos), currentRotation), 
      new PathPoint(new Translation2d((currentXPos + xTrans)/2, (currentYPos + yTrans)/2), currentRotation),
      new PathPoint(new Translation2d(xTrans, yTrans), currentRotation));
    
    return path2Target;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentPose = driveBase.getPose();
    
    frame = camera.getLatestResult();
    if(frame.hasTargets()){
      target = frame.getBestTarget();
    }
  }
}
