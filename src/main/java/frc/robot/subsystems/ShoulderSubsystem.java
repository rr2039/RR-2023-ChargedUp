// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {

  CANSparkMax rightShoulder;
  CANSparkMax leftShoulder;

  RelativeEncoder shoulderEnc;

  SparkMaxPIDController shoulderPID;
  
  /** Creates a new ArmSubsystem. */
  public ShoulderSubsystem() {

    rightShoulder = new CANSparkMax(ArmConstants.kRightShoulderCanId, MotorType.kBrushless);
    leftShoulder = new CANSparkMax(ArmConstants.kLeftShoulderCanId, MotorType.kBrushless);

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    //rightShoulder.setInverted(true);

    leftShoulder.follow(rightShoulder, true);

    //rightExtendyGirl.setInverted(true);
    //rightExtendyBoy.setInverted(true);
    //rightWristPitch.setInverted(true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    
    shoulderEnc = rightShoulder.getEncoder();

    shoulderPID = rightShoulder.getPIDController();

    shoulderPID.setFF(1);

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();
   
  }

  public void moveShoulderToPosition(double degrees) {
    shoulderPID.setReference(degrees, ControlType.kPosition);
  }

  public void moveShoudler(double speed) {
    rightShoulder.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
