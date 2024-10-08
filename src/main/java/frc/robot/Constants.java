// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Transform2d APRILTAG_CUBE_SCORE = new Transform2d(new Translation2d(1.25, 0), Rotation2d.fromDegrees(180));

  //public static final Pose3d APRILTAG_6_POS = new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0)));
  public static final Pose2d APRILTAG_6_POS = new Pose2d(1.02743, 4.424426, Rotation2d.fromDegrees(0));

  public static final class LimelightConstants {
    public static final double ledOff = 1;
    public static final double ledOn = 3;
    public static final double limelightDetectionDistance = 3;
  }

  public static final class GripperConstants {
    public static final int kPrimary = 0;
    public static final int kSecondary = 2;
  }

  public static final class ArmConstants {
    public static final int kRightShoulderCanId = 26;
    public static final int kLeftShoulderCanId = 25;
    public static final int kRightExtendyGirlCanId = 32;
    public static final int kLeftExtendyGirlCanId = 33;
    public static final int kRightExtendyBoyCanId = 34;
    public static final int kLeftExtendyBoyCanId = 35; 
    public static final int kRightWristPitchCanId = 36;
    public static final int kLeftWristPitchCanId = 37;
    public static final int kWristRollCanId = 38;

    public static final double kShoulderP = 0.015;
    public static final double kShoulderI = 0;
    public static final double kShoulderD = 1.0;
    public static final double kShoulderFF = 0;
    public static final double kShoulderGravity = -0.32;
    public static final double kShoulderStage1 = 0.0;
    public static final double kShoulderStage2 = 0.0;

    public static final double kWristPitchP = 0.025;
    public static final double kWristPitchI = 0;
    public static final double kWristPitchD = 0;
    public static final double kWristPitchFF = 0;

    public static final double kBoysP = 0.6;
    public static final double kBoysI = 0;
    public static final double kBoysD = 0;
    public static final double kBoysFF = 0;

    public static final double kGirlsP = 0.4;
    public static final double kGirlsI = 0;
    public static final double kGirlsD = 0;
    public static final double kGirlsFF = 0;

    public static final double kShoulderStartPoint = 0;
    public static final double kExtendyBoyStartPoint = -1;
    public static final double kExtendyGirlStartPoint = -1;
    public static final double kWristStartPoint = -36;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(30.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; //-Math.PI / 2
    public static final double kFrontRightChassisAngularOffset = 0; //0
    public static final double kBackLeftChassisAngularOffset = Math.PI; //Math.PI
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; //Math.PI / 2

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 2;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
