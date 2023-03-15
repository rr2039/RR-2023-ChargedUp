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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperPitchSubsystem extends SubsystemBase {
  CANSparkMax rightWristPitch;
  CANSparkMax leftWristPitch;

  RelativeEncoder wristPitchEnc;

  SparkMaxPIDController wristPitchPID;

  ShuffleboardTab gripperPitchTab = Shuffleboard.getTab("Arm");
  GenericEntry gripperPitchPos;
  GenericEntry gripperSetPoint;
  GenericEntry gripperPitchP;
  GenericEntry gripperPitchI;
  GenericEntry gripperPitchD;
  GenericEntry gripperPitchFF;
  
  /** Creates a new GripperPitchSubsystem. */
  public GripperPitchSubsystem() {

    rightWristPitch = new CANSparkMax(ArmConstants.kRightWristPitchCanId, MotorType.kBrushless);
    leftWristPitch = new CANSparkMax(ArmConstants.kLeftWristPitchCanId, MotorType.kBrushless);

    rightWristPitch.restoreFactoryDefaults();
    leftWristPitch.restoreFactoryDefaults();

    rightWristPitch.setSoftLimit(SoftLimitDirection.kForward, 30);
    rightWristPitch.setSoftLimit(SoftLimitDirection.kReverse, -40);

    leftWristPitch.follow(rightWristPitch, true);

    rightWristPitch.setIdleMode(IdleMode.kBrake);
    leftWristPitch.setIdleMode(IdleMode.kBrake);

    wristPitchEnc = rightWristPitch.getEncoder();
    wristPitchEnc.setPosition(0);
    gripperPitchPos = gripperPitchTab.add("Gripper Pitch Pos", getWristPitchPos()).getEntry();

    wristPitchPID = rightWristPitch.getPIDController();
    wristPitchPID.setP(ArmConstants.kShoulderP);
    gripperPitchP = gripperPitchTab.add("GripperWP", wristPitchPID.getP()).getEntry();
    wristPitchPID.setI(ArmConstants.kShoulderI);
    gripperPitchI = gripperPitchTab.add("GripperWI", wristPitchPID.getI()).getEntry();
    wristPitchPID.setD(ArmConstants.kShoulderD);
    gripperPitchD = gripperPitchTab.add("GripperWD", wristPitchPID.getD()).getEntry();
    wristPitchPID.setFF(ArmConstants.kShoulderFF);
    gripperPitchFF = gripperPitchTab.add("GripperWFF", wristPitchPID.getFF()).getEntry();

    rightWristPitch.burnFlash();
    leftWristPitch.burnFlash();

    gripperSetPoint = gripperPitchTab.add("GripperW Setpoint", 0).getEntry();
  }

  public void moveWristPitch(double speed) {
    rightWristPitch.set(speed);
  }

  public void moveWristPitchToPos() {
    wristPitchPID.setReference(getWristPitchPos(), ControlType.kPosition);
  }

  public double getWristPitchPos() {
    return wristPitchEnc.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gripperPitchPos.setDouble(getWristPitchPos());
    gripperSetPoint.getDouble(0);
    double tempP = gripperPitchP.getDouble(wristPitchPID.getP());
    if (wristPitchPID.getP() != tempP) {
      wristPitchPID.setP(tempP);
    }
    double tempI = gripperPitchI.getDouble(wristPitchPID.getI());
    if (wristPitchPID.getI() != tempI) {
      wristPitchPID.setI(tempI);
    }
    double tempD = gripperPitchD.getDouble(wristPitchPID.getD());
    if (wristPitchPID.getD() != tempD) {
      wristPitchPID.setD(tempD);
    }
    double tempFF = gripperPitchFF.getDouble(wristPitchPID.getFF());
    if (wristPitchPID.getFF() != tempFF) {
      wristPitchPID.setFF(tempFF);
    }
  }
}
