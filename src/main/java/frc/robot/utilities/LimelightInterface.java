// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightInterface extends SubsystemBase {
  NetworkTable n_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  boolean hasTarget;
  double aprilTagId;
  Pose2d aprilTagPosRelRobot;

  DriveSubsystem m_robotDrive;
  
  /** Creates a new LimelightInterface. */
  public LimelightInterface(DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hasTarget = n_limelight.getEntry("tv").getDouble(0) == 0 ? false : true;
    if (hasTarget) {
      aprilTagId = n_limelight.getEntry("tid").getDouble(0);
      double[] botposeDouble = n_limelight.getEntry("botpose").getDoubleArray(new double[]{0.0,0.0,0.0,0.0,0.0,0.0});
      Pose2d botpose = new Pose2d(botposeDouble[0], botposeDouble[1], new Rotation2d(botposeDouble[5]));
      m_robotDrive.feedVisionToPose(botpose);
      double[] targetPoseDouble = n_limelight.getEntry("botpose_targetspace").getDoubleArray(new double[]{0.0,0.0,0.0,0.0,0.0,0.0});
      aprilTagPosRelRobot = new Pose2d(targetPoseDouble[0], targetPoseDouble[1], new Rotation2d(targetPoseDouble[5]));
    }
  }

  public Pose2d getAprilTagPos() {
    return aprilTagPosRelRobot;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double aprilTabId() {
    return aprilTagId;
  }

  public void setLedStatus(double status) {
    n_limelight.getEntry("ledMode").setNumber(status);
  }
}
