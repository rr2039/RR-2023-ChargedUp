// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

  PneumaticHub m_pH = new PneumaticHub(2);
  Solenoid primaryPistons;
  Solenoid secondaryPistons; 

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    primaryPistons = m_pH.makeSolenoid(1);
    secondaryPistons = m_pH.makeSolenoid(0);
  }

  public void open() {
    primaryPistons.set(true);
    secondaryPistons.set(false);
  }

  public void hardClose() {
    primaryPistons.set(false);
    secondaryPistons.set(true);
  }

  public void softClose() {
    primaryPistons.set(false);
    secondaryPistons.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
