// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class TransportPosition extends Command {

  ShoulderSubsystem shoulder;
  GripperPitchSubsystem gripper;
  ArmExtensionSubsystem arm;

  /** Creates a new TransportPosition. */
  public TransportPosition(ShoulderSubsystem m_shoulder, GripperPitchSubsystem m_gripperPitch, ArmExtensionSubsystem m_arm) {
    shoulder = m_shoulder;
    gripper = m_gripperPitch;
    arm = m_arm;
   // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(m_shoulder, m_gripperPitch, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shoulder.getScoreMode() == 1) {
      shoulder.setShoulderCurSetpoint(0);
      gripper.setWristCurSetpoint(-16);
      arm.setExtendyBoyCurSetpoint(0);
      arm.setExtendyGirlCurSetpoint(0);
    } else {
      shoulder.setShoulderCurSetpoint(0);
      gripper.setWristCurSetpoint(26);
      arm.setExtendyBoyCurSetpoint(0);
      arm.setExtendyGirlCurSetpoint(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
