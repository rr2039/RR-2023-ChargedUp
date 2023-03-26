// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightInterface extends SubsystemBase {
  //NetworkTable n_limelight = NetworkTableInstance.getDefault().getTable("limelight");

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
    //[0.029571,-0.080881,0.644705,-16.887242,-1.438812,1.242106]

    hasTarget = LimelightHelpers.getTV("");
    if (hasTarget) {
      aprilTagId = LimelightHelpers.getFiducialID("");
      aprilTagPosRelRobot = LimelightHelpers.toPose2DFixLimelightsGlaringMistake(LimelightHelpers.getTargetPose_RobotSpace(""));
      aprilTagField.setRobotPose(aprilTagPosRelRobot);
      Pose2d botpose = LimelightHelpers.getBotPose2d_wpiBlue("");
      llBotField.setRobotPose(botpose);
      SmartDashboard.putNumber("AprilTagID", aprilTagId);
      SmartDashboard.putBoolean("HasTarget", hasTarget);
      //SmartDashboard.putString("AprilTagPosRelBot", aprilTagPosRelRobot.toString());
      //SmartDashboard.putData("AprilTagFieldPos", aprilTagField);
      //SmartDashboard.putString("LLBotPose", botpose.toString());
      //SmartDashboard.putData("LLBotPoseFieldPos", llBotField);
      if ((aprilTagPosRelRobot.getX() < LimelightConstants.limelightDetectionDistance 
              && aprilTagPosRelRobot.getX() > -LimelightConstants.limelightDetectionDistance) 
              && (aprilTagPosRelRobot.getY() < LimelightConstants.limelightDetectionDistance 
              && aprilTagPosRelRobot.getY() > -LimelightConstants.limelightDetectionDistance)) {
        m_robotDrive.feedVisionToPose(botpose);
      }
    }
  }
}
