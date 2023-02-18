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

public class ArmSubsystem extends SubsystemBase {

  CANSparkMax rightShoulder;
  CANSparkMax leftShoulder;
  CANSparkMax rightExtendyGirl;
  CANSparkMax leftExtendyGirl; 
  CANSparkMax rightExtendyBoy;
  CANSparkMax leftExtendyBoy;
  CANSparkMax rightWristPitch;
  CANSparkMax leftWristPitch;
  CANSparkMax wristRoll; 

  RelativeEncoder shoulderEnc;
  RelativeEncoder extendyGirlEnc;
  RelativeEncoder extendyBoyEnc;
  RelativeEncoder wristPitchEnc;
  RelativeEncoder wristRollEnc;

  SparkMaxPIDController shoulderPID;
  SparkMaxPIDController extendyGirlPID;
  SparkMaxPIDController extendyBoyPID;
  SparkMaxPIDController wristPitchPID;
  SparkMaxPIDController wristRollPID;
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    rightShoulder = new CANSparkMax(ArmConstants.kRightShoulderCanId, MotorType.kBrushless);
    leftShoulder = new CANSparkMax(ArmConstants.kLeftShoulderCanId, MotorType.kBrushless);
    rightExtendyGirl = new CANSparkMax(ArmConstants.kRightExtendyGirlCanId, MotorType.kBrushless);
    leftExtendyGirl = new CANSparkMax(ArmConstants.kLeftExtendyGirlCanId, MotorType.kBrushless);
    rightExtendyBoy = new CANSparkMax(ArmConstants.kRightExtendyBoyCanId, MotorType.kBrushless);
    leftExtendyBoy = new CANSparkMax(ArmConstants.kLeftExtendyBoyCanId, MotorType.kBrushless);
    rightWristPitch = new CANSparkMax(ArmConstants.kRightWristPitchCanId, MotorType.kBrushless);
    leftWristPitch = new CANSparkMax(ArmConstants.kLeftWristPitchCanId, MotorType.kBrushless);
    wristRoll = new CANSparkMax(ArmConstants.kWristRollCanId, MotorType.kBrushless);

    leftShoulder.follow(rightShoulder);
    leftExtendyGirl.follow(rightExtendyGirl);
    leftExtendyBoy.follow(rightExtendyBoy);
    leftWristPitch.follow(rightWristPitch);

    rightShoulder.setInverted(true);
    rightExtendyGirl.setInverted(true);
    rightExtendyBoy.setInverted(true);
    rightWristPitch.setInverted(true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    rightExtendyGirl.setIdleMode(IdleMode.kBrake);
    leftExtendyGirl.setIdleMode(IdleMode.kBrake);
    rightExtendyBoy.setIdleMode(IdleMode.kBrake);
    leftExtendyBoy.setIdleMode(IdleMode.kBrake);
    rightWristPitch.setIdleMode(IdleMode.kBrake);
    leftWristPitch.setIdleMode(IdleMode.kBrake);
    wristRoll.setIdleMode(IdleMode.kBrake);

    shoulderEnc = rightShoulder.getEncoder();
    extendyGirlEnc = rightExtendyGirl.getEncoder();
    extendyBoyEnc = rightExtendyBoy.getEncoder();
    wristPitchEnc = rightWristPitch.getEncoder();
    wristRollEnc = wristRoll.getEncoder();

    shoulderPID = rightShoulder.getPIDController();
    extendyGirlPID = rightExtendyGirl.getPIDController();
    extendyBoyPID = rightExtendyBoy.getPIDController();
    wristPitchPID = rightWristPitch.getPIDController();
    wristRollPID = wristRoll.getPIDController();

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();
    rightExtendyGirl.burnFlash();
    leftExtendyGirl.burnFlash();
    rightExtendyBoy.burnFlash();
    leftExtendyBoy.burnFlash();
    rightWristPitch.burnFlash();
    leftWristPitch.burnFlash();
    wristRoll.burnFlash();
  }

  public void moveShoulderToPosition(double degrees) {
    shoulderPID.setReference(degrees, ControlType.kPosition);
  }

  public void moveShoudler(double speed) {
    rightShoulder.set(speed);
  }

  public void moveExtendyGirls(double speed) {
    rightExtendyGirl.set(speed);
  }
  
  public void moveExtendyBoys(double speed) {
    rightExtendyBoy.set(speed);
  }

  public void moveWristPitch(double speed) {
    rightWristPitch.set(speed);
  }

  public void moveWristRoll(double speed) {
    wristRoll.set(speed);
  }

  public void moveExtendors(double speed) {
    rightExtendyGirl.set(speed);
    rightExtendyBoy.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
