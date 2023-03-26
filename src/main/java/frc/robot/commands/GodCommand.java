// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.PresetPositions.TransportPosition;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class GodCommand extends CommandBase {

  ShoulderSubsystem shoulder;
  GripperPitchSubsystem gripper;
  ArmExtensionSubsystem arm;
  GenericHID operatorContoller;

  SlewRateLimiter boySlew = new SlewRateLimiter(9);
  SlewRateLimiter girlSlew = new SlewRateLimiter(9);
  SlewRateLimiter wristSlew = new SlewRateLimiter(180);
  SlewRateLimiter shoulderSlew = new SlewRateLimiter(270);
  
  /** Creates a new GodCommand. */
  public GodCommand(ShoulderSubsystem m_shoulder, GripperPitchSubsystem m_gripperPitch, ArmExtensionSubsystem m_arm, GenericHID m_operatorController) {
    shoulder = m_shoulder;
    gripper = m_gripperPitch;
    arm = m_arm;
    operatorContoller = m_operatorController;
   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoulder, m_gripperPitch, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boySlew.calculate(-1);
    girlSlew.calculate(-1);
    shoulderSlew.calculate(0);
    new TransportPosition(shoulder, gripper, arm).execute();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Manual Controls?
    // Move arm in
    if (operatorContoller.getRawButtonPressed(5)) {
      arm.setExtendyBoyCurSetpoint(arm.getExtendyBoyCurSetpoint() - 0.5);
      arm.setExtendyGirlCurSetpoint(arm.getExtendyGirlCurSetpoint() - 0.5);
    }
    // Move arm out
    if (operatorContoller.getRawButtonPressed(6)) {
      arm.setExtendyBoyCurSetpoint(arm.getExtendyBoyCurSetpoint() + 0.5);
      arm.setExtendyGirlCurSetpoint(arm.getExtendyGirlCurSetpoint() + 0.5);
    }
    // Move wrist
    if (Math.abs(operatorContoller.getRawAxis(3)) > 0.5) {
      gripper.setWristCurSetpoint(gripper.getWristCurSetpoint() + (operatorContoller.getRawAxis(3) > 0 ? -1 : 1));
    }
    // Move shoulder
    if (Math.abs(operatorContoller.getRawAxis(1)) > 0.5) {
      shoulder.setShoulderCurSetpoint(shoulder.getShoulderCurSetpoint() + (operatorContoller.getRawAxis(1) > 0 ? 1 : -1));
    }

    double shoulderArbFF = Math.sin(Math.toRadians(shoulder.getShoulderPos()))
                        * ((ArmConstants.kShoulderGravity 
                        + (arm.getExtendyBoyPos() * ArmConstants.kShoulderStage1) 
                        + (arm.getExtendyGirlPos() * ArmConstants.kShoulderStage2)));
    double boysVelocity;
    if (Math.abs(arm.getExtendyBoyCurSetpoint() - arm.getExtendyBoyPos()) < 0.125) {
      boysVelocity = 0;
    } else {
      boysVelocity = arm.getExtendyBoyCurSetpoint() - arm.getExtendyBoyPos() > 0 ? 1 : -1;
    }
    double girlsVelocity;
    if (Math.abs(arm.getExtendyGirlCurSetpoint() - arm.getExtendyGirlPos()) < 0.125) {
      girlsVelocity = 0;
    } else {
      girlsVelocity = arm.getExtendyGirlCurSetpoint() - arm.getExtendyGirlPos() > 0 ? 1 : -1;
    }
    double boysArbFF = (Math.cos(Math.toRadians(shoulder.getShoulderPos())) * 0.2) 
                          + (Math.signum(boysVelocity) * 0.5);
    double girlsArbFF = (Math.cos(Math.toRadians(shoulder.getShoulderPos())) * 0.1) 
                          + (Math.signum(girlsVelocity) * 0.3);
    shoulder.moveShoulderToPosition(shoulderSlew.calculate(shoulder.getShoulderCurSetpoint()), shoulderArbFF);
    gripper.moveWristPitchToPos(wristSlew.calculate(gripper.getWristCurSetpoint()));
    arm.moveExtendyBoysToPos(boySlew.calculate(arm.getExtendyBoyCurSetpoint()), boysArbFF);
    arm.moveExtendyGirlsToPos(girlSlew.calculate(arm.getExtendyGirlCurSetpoint()), girlsArbFF);
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
