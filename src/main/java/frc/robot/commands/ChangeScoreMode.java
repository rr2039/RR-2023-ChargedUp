// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.utilities.LEDController;

public class ChangeScoreMode extends CommandBase {

  ShoulderSubsystem shoulder;
  LEDController led;
  int mode;

  /** Creates a new ChangeScoreMode. */
  public ChangeScoreMode(ShoulderSubsystem m_shoulder, LEDController m_led, int m_mode) {
    shoulder = m_shoulder;
    led = m_led;
    mode = m_mode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setScoreMode(mode);
    led.setGamePiece(mode);
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
