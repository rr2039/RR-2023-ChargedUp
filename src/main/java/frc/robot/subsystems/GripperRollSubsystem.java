// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

public class GripperRollSubsystem extends SubsystemBase {

  CANSparkMax wristRoll;

  RelativeEncoder wristRollEnc;

  SparkMaxPIDController wristRollPID;


  /** Creates a new GripperRollSubsystem. */
  public GripperRollSubsystem() {

    wristRoll = new CANSparkMax(ArmConstants.kWristRollCanId, MotorType.kBrushless);

    wristRoll.restoreFactoryDefaults();

    wristRoll.setIdleMode(IdleMode.kBrake);

    wristRollEnc = wristRoll.getEncoder();
    wristRollEnc.setPositionConversionFactor(1.719);

    wristRollPID = wristRoll.getPIDController();

    wristRoll.burnFlash();

  }

  public void moveWristRoll(double speed) {
    wristRoll.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
