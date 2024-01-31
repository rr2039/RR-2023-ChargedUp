// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;

public class ArmExtension extends CommandBase {
  ArmExtensionSubsystem arm = null;
  double speed = 0;
  boolean extendyBoysStop = false;
  boolean extendyGirlsStop = false;
  /** Creates a new ArmExtension. */
  public ArmExtension(ArmExtensionSubsystem m_arm, double m_speed) {
    arm = m_arm;
    speed = m_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extendyBoysStop = false;
    extendyGirlsStop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extendyBoysStop == false) {
      arm.moveExtendyBoys(speed);
    }
    if (extendyGirlsStop == false) {
      arm.moveExtendyGirls(-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((speed < 0 && arm.getExtendyBoyPos() <= 0) || speed == 0) {
      arm.moveExtendyBoys(0);
      extendyBoysStop = true;
    } else if ((speed > 0 && arm.getExtendyBoyPos() >= 120) || speed == 0) {
      arm.moveExtendyBoys(0);
      extendyBoysStop = true;
    }
    if ((speed < 0 && arm.getExtendyGirlPos() <= 0) || speed == 0) {
      arm.moveExtendyGirls(0);
      extendyGirlsStop = true;
    } else if ((speed > 0 && arm.getExtendyGirlPos() >= 120) || speed == 0) {
      arm.moveExtendyGirls(0);
      extendyGirlsStop = true;
    }
    System.out.println("Girls " + extendyGirlsStop);
    System.out.println("Boys " + extendyBoysStop);
    return extendyBoysStop == true && extendyGirlsStop == true;
  }
}
