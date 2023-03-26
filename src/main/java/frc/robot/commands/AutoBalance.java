// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {

  DriveSubsystem drive;
  boolean climbing = false;
  double fallAngle = 14.0;
  double time = 0;
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem m_drive) {
    drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbing = false;
    time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drive.getPitch() > -fallAngle && drive.getPitch() < fallAngle && !climbing) {
      drive.drive(-0.2, 0, 0, false);
    } else if (drive.getPitch() > -fallAngle && drive.getPitch() < fallAngle && climbing) {
      drive.drive(0, 0, 0, false);
      drive.setX();
    } else if ((drive.getPitch() < -fallAngle || drive.getPitch() > fallAngle) && !climbing && time > 500) {
      climbing = true;
    } else if (drive.getPitch() > fallAngle && climbing) {
      drive.drive(drive.getPitch()/75, 0, 0, false);
    } else if (drive.getPitch() < -fallAngle && climbing) {
      drive.drive(drive.getPitch()/75, 0, 0, false);
    }
    time += 1;
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
