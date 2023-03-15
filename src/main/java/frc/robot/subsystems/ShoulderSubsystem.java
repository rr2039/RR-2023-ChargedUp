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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {

  CANSparkMax rightShoulder;
  CANSparkMax leftShoulder;

  RelativeEncoder shoulderEnc;

  SparkMaxPIDController shoulderPID;

  ShuffleboardTab shoulderTab = Shuffleboard.getTab("Arm");
  
  /** Creates a new ArmSubsystem. */
  public ShoulderSubsystem() {

    rightShoulder = new CANSparkMax(ArmConstants.kRightShoulderCanId, MotorType.kBrushless);
    leftShoulder = new CANSparkMax(ArmConstants.kLeftShoulderCanId, MotorType.kBrushless);

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    leftShoulder.follow(rightShoulder, true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    
    shoulderEnc = rightShoulder.getEncoder();
    shoulderEnc.setPosition(0);

    shoulderPID = rightShoulder.getPIDController();
    shoulderPID.setP(ArmConstants.kShoulderP);
    shoulderPID.setI(ArmConstants.kShoulderI);
    shoulderPID.setD(ArmConstants.kShoulderD);
    shoulderPID.setFF(ArmConstants.kShoulderFF);

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();
  }

  public void moveShoulderToPosition(double degrees) {
    shoulderPID.setReference(degrees, ControlType.kPosition);
  }

  public void moveShoudler(double speed) {
    rightShoulder.set(speed);
  }

  public double getShoulderPos() {
    return shoulderEnc.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderTab.add("ShoulderPos", getShoulderPos());
    GenericEntry tempP = shoulderTab.add("Shoulder P", ArmConstants.kShoulderP).getEntry();
    if (shoulderPID.getP() != tempP.getDouble(ArmConstants.kShoulderP)) {
      shoulderPID.setP(tempP.getDouble(ArmConstants.kShoulderP));
    }
    GenericEntry tempI = shoulderTab.add("Shoulder I", ArmConstants.kShoulderI).getEntry();
    if (shoulderPID.getI() != tempI.getDouble(ArmConstants.kShoulderI)) {
      shoulderPID.setI(tempI.getDouble(ArmConstants.kShoulderI));
    }
    GenericEntry tempD = shoulderTab.add("Shoulder D", ArmConstants.kShoulderD).getEntry();
    if (shoulderPID.getD() != tempD.getDouble(ArmConstants.kShoulderD)) {
      shoulderPID.setD(tempD.getDouble(ArmConstants.kShoulderD));
    }
    GenericEntry tempFF = shoulderTab.add("Shoulder FF", ArmConstants.kShoulderFF).getEntry();
    if (shoulderPID.getFF() != tempFF.getDouble(ArmConstants.kShoulderFF)) {
      shoulderPID.setFF(tempFF.getDouble(ArmConstants.kShoulderFF));
    }
  }
}
