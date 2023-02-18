// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {

  DoubleSolenoid primaryPistons;
  Solenoid secondaryPistons; 

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    primaryPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GripperConstants.kPrimaryForward, GripperConstants.kPrimaryReverse);
    secondaryPistons = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.kSecondary);
  }

  public void open() {
    primaryPistons.set(Value.kReverse);
    secondaryPistons.set(false);
  }

  public void hardClose() {
    primaryPistons.set(Value.kForward);
    secondaryPistons.set(true);
  }

  public void softClose() {
    primaryPistons.set(Value.kForward);
    secondaryPistons.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
