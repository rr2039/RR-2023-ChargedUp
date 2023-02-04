// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightInterface extends SubsystemBase {
  NetworkTable n_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  boolean hasTarget;
  double aprilTagId;
  Pose2d aprilTagPosRelRobot;

  DriveSubsystem m_robotDrive;
  
	private final Field2d aprilTagField = new Field2d();
	private final Field2d llBotField = new Field2d();

  /** Creates a new LimelightInterface. */
  public LimelightInterface(DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hasTarget = n_limelight.getEntry("tv").getDouble(0) == 0 ? false : true;
		SmartDashboard.putBoolean("HasTarget", hasTarget);
    //System.out.println(hasTarget);
    if (hasTarget) {
      aprilTagId = n_limelight.getEntry("tid").getDouble(0);
			SmartDashboard.putNumber("AprilTagID", aprilTagId);
      double[] targetPoseDouble = n_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
      if (targetPoseDouble.length > 0) {
        aprilTagPosRelRobot = new Pose2d(targetPoseDouble[2], targetPoseDouble[0], Rotation2d.fromDegrees(targetPoseDouble[4]));
        SmartDashboard.putString("AprilTagPosRelBot", aprilTagPosRelRobot.toString());
				aprilTagField.setRobotPose(aprilTagPosRelRobot);
				SmartDashboard.putData("AprilTagFieldPos", aprilTagField);
        if ((aprilTagPosRelRobot.getX() < LimelightConstants.limelightDetectionDistance 
              && aprilTagPosRelRobot.getX() > -LimelightConstants.limelightDetectionDistance) 
              && (aprilTagPosRelRobot.getY() < LimelightConstants.limelightDetectionDistance 
              && aprilTagPosRelRobot.getY() > -LimelightConstants.limelightDetectionDistance)) {
          double[] botposeDouble = n_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
          if (botposeDouble.length > 0) {
            Pose2d botpose = new Pose2d(botposeDouble[0], botposeDouble[1], Rotation2d.fromDegrees(botposeDouble[5]));
            SmartDashboard.putString("LLBotPose", botpose.toString());
						llBotField.setRobotPose(botpose);
						SmartDashboard.putData("LLBotPoseFieldPos", llBotField);
            m_robotDrive.feedVisionToPose(botpose);
          }
        }
      }
    }
  }

  public Pose2d getAprilTagPos() {
    return aprilTagPosRelRobot;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double aprilTagId() {
    return aprilTagId;
  }

  public void setLedStatus(double status) {
    n_limelight.getEntry("ledMode").setNumber(status);
  }
}
