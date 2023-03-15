// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class TransportPosition extends CommandBase {

  ShoulderSubsystem shoulder;
  GripperPitchSubsystem gripper;

  /** Creates a new TransportPosition. */
  public TransportPosition(ShoulderSubsystem m_shoulder, GripperPitchSubsystem m_gripperPitch) {
    shoulder = m_shoulder;
    gripper = m_gripperPitch;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoulder, m_gripperPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.moveShoulderToPosition(0);
    gripper.moveWristPitchToPos(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
