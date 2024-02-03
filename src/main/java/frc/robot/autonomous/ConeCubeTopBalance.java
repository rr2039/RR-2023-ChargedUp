// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.PresetPositions.FloorPickup;
import frc.robot.commands.PresetPositions.HighScore;
import frc.robot.commands.PresetPositions.MediumScore;
import frc.robot.commands.PresetPositions.TransportPosition;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeCubeTopBalance extends SequentialCommandGroup {
  /** Creates a new ConeCubeTopBalance. */
  public ConeCubeTopBalance(DriveSubsystem m_robotDrive, ShoulderSubsystem m_shoulder, GripperPitchSubsystem m_gripper, ArmExtensionSubsystem m_arm, GripperSubsystem m_claw) {

    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ConeCubeTopBalance", new PathConstraints(3, 2), new PathConstraints(3, 2));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("floorpickup", new FloorPickup(m_shoulder, m_gripper, m_arm, m_claw));
    eventMap.put("grabpiece", new RunCommand(()-> m_claw.hardClose(), m_claw)).withTimeout(0.5).andThen(new TransportPosition(m_shoulder, m_gripper, m_arm).andThen(new RunCommand(() -> m_shoulder.setScoreMode(1), m_shoulder)));
    eventMap.put("highscore", new HighScore(m_shoulder, m_gripper, m_arm));
    //eventMap.put("cubescore", new RunCommand(() -> m_claw.open(), m_claw).withTimeout(0.5));

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

    Command poseReset = autoBuilder.resetPose(pathGroup.get(0));
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(poseReset,
                new RunCommand(() -> m_shoulder.setScoreMode(0), m_shoulder),
                new HighScore(m_shoulder, m_gripper, m_arm),
                new WaitCommand(3),
                new RunCommand(() -> m_claw.open(), m_claw).withTimeout(0.1),
                new WaitCommand(0.5),
                new TransportPosition(m_shoulder, m_gripper, m_arm),
                autoBuilder.followPath(pathGroup.get(0)),
                new WaitCommand(0.5),
                autoBuilder.followPath(pathGroup.get(1)),
                new RunCommand(() -> m_claw.open(), m_claw).withTimeout(0.5),
                new WaitCommand(0.5),
                autoBuilder.followPath(pathGroup.get(2)),
                new AutoBalance(m_robotDrive));
  }
}
