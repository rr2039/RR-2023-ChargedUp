// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightInterface extends SubsystemBase {
  //NetworkTable n_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  boolean hasTarget1;
  boolean hasTarget2;
  double[] aprilTagIds;
  Pose2d aprilTagPosRelRobot;
  double tag1;
  double tag2;

  String limelight1 = "limelight-three";
  String limelight2 = "limelight-twoplus";

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

    Alliance alliance = DriverStation.getAlliance().get();

    hasTarget1 = LimelightHelpers.getTV(limelight1);
    tag1 = LimelightHelpers.getFiducialID(limelight1);
    if (hasTarget1) {
      aprilTagPosRelRobot = LimelightHelpers.toPose2DFixLimelightsGlaringMistake(LimelightHelpers.getTargetPose_RobotSpace(limelight1));
      //aprilTagField.setRobotPose(aprilTagPosRelRobot);
      Pose2d botpose;
      if (alliance == Alliance.Blue) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(limelight1);
      } else {
        botpose = LimelightHelpers.getBotPose2d_wpiRed(limelight1);
      }
      //llBotField.setRobotPose(botpose);
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
    hasTarget2 = LimelightHelpers.getTV(limelight2);
    tag2 = LimelightHelpers.getFiducialID(limelight2);
    if (hasTarget2) {
      aprilTagPosRelRobot = LimelightHelpers.toPose2DFixLimelightsGlaringMistake(LimelightHelpers.getTargetPose_RobotSpace(limelight1));
      Pose2d botpose;
      if (alliance == Alliance.Blue) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(limelight1);
      } else {
        botpose = LimelightHelpers.getBotPose2d_wpiRed(limelight1);
      }
      if ((aprilTagPosRelRobot.getX() < LimelightConstants.limelightDetectionDistance 
              && aprilTagPosRelRobot.getX() > -LimelightConstants.limelightDetectionDistance) 
              && (aprilTagPosRelRobot.getY() < LimelightConstants.limelightDetectionDistance 
              && aprilTagPosRelRobot.getY() > -LimelightConstants.limelightDetectionDistance)) {
        m_robotDrive.feedVisionToPose(botpose);
      }
    }
    aprilTagIds = new double[]{tag1, tag2};
    SmartDashboard.putString("AprilTagID", Arrays.toString(aprilTagIds));
    SmartDashboard.putBoolean("HasTarget", hasTarget1 || hasTarget2);
  }
}
