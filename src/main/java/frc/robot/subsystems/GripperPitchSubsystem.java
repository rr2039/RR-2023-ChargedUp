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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperPitchSubsystem extends SubsystemBase {
  CANSparkMax rightWristPitch;
  CANSparkMax leftWristPitch;

  RelativeEncoder wristPitchEnc;

  SparkMaxPIDController wristPitchPID;

  ShuffleboardTab gripperPitchTab = Shuffleboard.getTab("Arm");
  
  /** Creates a new GripperPitchSubsystem. */
  public GripperPitchSubsystem() {

    rightWristPitch = new CANSparkMax(ArmConstants.kRightWristPitchCanId, MotorType.kBrushless);
    leftWristPitch = new CANSparkMax(ArmConstants.kLeftWristPitchCanId, MotorType.kBrushless);

    rightWristPitch.restoreFactoryDefaults();
    leftWristPitch.restoreFactoryDefaults();

    leftWristPitch.follow(rightWristPitch, true);

    rightWristPitch.setIdleMode(IdleMode.kBrake);
    leftWristPitch.setIdleMode(IdleMode.kBrake);

    wristPitchEnc = rightWristPitch.getEncoder();
    wristPitchEnc.setPosition(0);

    wristPitchPID = rightWristPitch.getPIDController();

    rightWristPitch.burnFlash();
    leftWristPitch.burnFlash();
  }

  public void moveWristPitch(double speed) {
    rightWristPitch.set(speed);
  }

  public double getWristPitchPos() {
    return wristPitchEnc.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gripperPitchTab.add("Gripper Pitch", getWristPitchPos());
  }
}
