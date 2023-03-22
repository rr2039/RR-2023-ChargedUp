// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.GodCommand;
import frc.robot.commands.PresetPositions.LowScore;
import frc.robot.commands.PresetPositions.TransportPosition;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeBottom extends SequentialCommandGroup {
  /** Creates a new CubeBottom. */
  public CubeBottom(DriveSubsystem m_robotDrive, ShoulderSubsystem m_shoulder, GripperPitchSubsystem m_gripper, ArmExtensionSubsystem m_arm, GripperSubsystem m_claw) {

    PathPlannerTrajectory path = PathPlanner.loadPath("CubeBottom", new PathConstraints(3, 2));
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose, // Pose2d supplier
      m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(6.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command poseReset = autoBuilder.resetPose(path);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(poseReset,
                //new GodCommand(m_shoulder, m_gripper, m_arm),
                new LowScore(m_shoulder, m_gripper, m_arm),
                new WaitCommand(2),
                new RunCommand(() -> m_claw.open(), m_claw).withTimeout(0.1),
                new WaitCommand(0.5),
                new TransportPosition(m_shoulder, m_gripper, m_arm),
                new WaitCommand(0.5),
                new RunCommand(() -> m_claw.softClose(), m_claw).withTimeout(0.1),
                autoBuilder.followPath(path));
  }
}
