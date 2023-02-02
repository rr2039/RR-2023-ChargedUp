// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.LimelightInterface;

public class AlignToAprilTag extends CommandBase {
  // Create config for trajectory
  TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  DriveSubsystem robotDrive;
  LimelightInterface limelight;
  SwerveControllerCommand swerveControllerCommand;

  /** Creates a new AlignToAprilTag. */
  public AlignToAprilTag(DriveSubsystem m_robotDrive, LimelightInterface m_limelight) {
    robotDrive = m_robotDrive;
    limelight = m_limelight;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get where april tag is
    Pose2d aprilTagPos = limelight.getAprilTagPos();
    // Find where that is in relation to robots current pose
    Pose2d endPoint = robotDrive.getPose().transformBy(new Transform2d(new Translation2d(aprilTagPos.getX(), aprilTagPos.getY()), aprilTagPos.getRotation()));
    // Last transform endpoint to be offset so bot doesn't try to ram through scoring
    endPoint.transformBy(Constants.APRILTAG_CUBE_SCORE);

    // Generate trajectory for swerve to follow based on where robot is and where tag should be. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // List of start point and end point
        List.of(robotDrive.getPose(), endPoint),
        config);

    // Generate swerve controller to move bot along trajectory
    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        robotDrive::setModuleStates,
        robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    swerveControllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveControllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveControllerCommand.end(interrupted);
    robotDrive.drive(0, 0, 0, false); // Stop robot driving, not sure if above line does
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveControllerCommand.isFinished();
  }
}
