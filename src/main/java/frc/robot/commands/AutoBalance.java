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
  boolean fell = false;
  double fallAngle = 12.0;
  double targetAngle = 5.0;
  double time = 0;
  boolean direction = false;
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem m_drive) {
    drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AutoBalance(DriveSubsystem m_drive, boolean m_direction) {
    drive = m_drive;
    direction = m_direction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbing = false;
    time = 0;
    fell = false;
    //fallAngle = 13.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (drive.getPitch() > -fallAngle && drive.getPitch() < fallAngle && !climbing && time < 5) {
    /*if (!climbing && time < 3) {
      // Start moving
      drive.drive((direction ? 0.2 : -0.2), 0, 0, false);
    } else if (-fallAngle < drive.getPitch() && drive.getPitch() < fallAngle && climbing) {
      // Stop moving
      drive.drive(0, 0, 0, false);
      drive.setX();
      //fallAngle -= 1;
    } else if ((drive.getPitch() < -fallAngle || fallAngle < drive.getPitch()) && !climbing && time > 3) {
      // Decide that were climbing
      climbing = true;
    } else if ((drive.getPitch() < -fallAngle || fallAngle < drive.getPitch()) && climbing) {
      // Move direction to balance 
      drive.drive(drive.getPitch()/150, 0, 0, false);
    }
    if (time < 3) {
      time += 0.02;
    }*/

    // Initialize climb
    if ((-fallAngle < drive.getPitch() || drive.getPitch() < fallAngle) && !climbing) {
      drive.drive((direction ? 0.2 : -0.2), 0, 0, false);
    }
    // Start climbing
    if ((drive.getPitch() < - fallAngle || fallAngle < drive.getPitch()) && !climbing && time > 3) {
      climbing = true;
    }
    if ((drive.getPitch() < - fallAngle || fallAngle < drive.getPitch()) && climbing) {
      drive.drive((direction ? 0.2 : -0.2), 0, 0, false);
    }
    // Taper climbing +
    if ((-fallAngle < drive.getPitch() || drive.getPitch() < fallAngle) && climbing) {
      //double speed = 0.2 - (drive.getPitch() - fallAngle)/(targetAngle - fallAngle);
      double speed = drive.getPitch()/75;
      drive.drive(speed, 0, 0, false);
    }
    if ((-targetAngle < drive.getPitch() || drive.getPitch() < targetAngle) && climbing) {
      drive.drive(0, 0, 0, false);
      drive.setX();
    }
    // stationary
    // taper climbing - 
    // full speed backwards
    if (time < 3) {
      time += 0.02;
    }
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
