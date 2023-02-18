// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

    shoulderEnc = rightShoulder.getEncoder();
    extendyGirlEnc = rightExtendyGirl.getEncoder();
    extendyBoyEnc = rightExtendyBoy.getEncoder();
    wristPitchEnc = rightWristPitch.getEncoder();
    wristRollEnc = wristRoll.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
