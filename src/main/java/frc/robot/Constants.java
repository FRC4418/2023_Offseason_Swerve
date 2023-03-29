// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class moduleConstants{
    public static final double kPhysicalMaxSpeedPerSeconds = 10.0;
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 8;
    public static final double kTurningMotorGearRatio = 1 / 8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameter;
    public static final double kTurningEncoderRot2Meter = kTurningMotorGearRatio * Math.PI * 2;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2MeterPerSec = kTurningEncoderRot2Meter / 60;
    public static final double kPTurning = 0.5;
  }
  public static class encoders{
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;
    
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;
  }
  public static class moduleTranslations{
    public static final Translation2d frontRightTrans = new Translation2d(0.5,0.5);
    public static final Translation2d frontLeftTrans = new Translation2d(0.5,0.5);
    public static final Translation2d backRightTrans = new Translation2d(0.5,0.5);
    public static final Translation2d backLeftTrans = new Translation2d(0.5,0.5);
  }
  public static class controllerConstants{
    public static final double deadBand = 0.3;
  }
  public static class DriveConstants{
    public static double kMaxAccelerationUnitsPerSecond = 1.0;
    public static double kMaxAngularAccelerationPerSecond = 1.0;
    public static double kMaxTeleOpSpeed = 2.0;
    public static double kMaxTeleOpAngularSpeed = 1.5;
    public static double kEncoderCPR = 2048;
  }
  public static class kinematics{
    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(25);
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase/ 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    );
  }
}
