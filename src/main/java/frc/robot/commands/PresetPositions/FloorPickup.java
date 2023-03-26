// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class FloorPickup extends CommandBase {

  ShoulderSubsystem shoulder;
  GripperPitchSubsystem gripper;
  ArmExtensionSubsystem arm;
  GripperSubsystem claw;

  /** Creates a new FloorPickup. */
  public FloorPickup(ShoulderSubsystem m_shoulder, GripperPitchSubsystem m_gripperPitch, ArmExtensionSubsystem m_arm, GripperSubsystem m_claw) {
    shoulder = m_shoulder;
    gripper = m_gripperPitch;
    arm = m_arm;
    claw = m_claw;
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
      shoulder.setShoulderCurSetpoint(120);
      gripper.setWristCurSetpoint(11);
      arm.setExtendyBoyCurSetpoint(0);
      arm.setExtendyGirlCurSetpoint(0);
    } else {
      shoulder.setShoulderCurSetpoint(115);
      gripper.setWristCurSetpoint(2.5);
      arm.setExtendyBoyCurSetpoint(0);
      arm.setExtendyGirlCurSetpoint(0);
    }
    claw.open();
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
