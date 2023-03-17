// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {

  CANSparkMax rightShoulder;
  CANSparkMax leftShoulder;

  RelativeEncoder shoulderEnc;

  SparkMaxPIDController shoulderPID;

  ShuffleboardTab shoulderTab = Shuffleboard.getTab("Arm");
  GenericEntry shoulderPos;
  GenericEntry shoulderSetpoint;
  GenericEntry shoulderP;
  GenericEntry shoulderI;
  GenericEntry shoulderD;
  GenericEntry shoulderFF;
  
  /** Creates a new ArmSubsystem. */
  public ShoulderSubsystem() {

    rightShoulder = new CANSparkMax(ArmConstants.kRightShoulderCanId, MotorType.kBrushless);
    leftShoulder = new CANSparkMax(ArmConstants.kLeftShoulderCanId, MotorType.kBrushless);

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    leftShoulder.enableVoltageCompensation(12.0);
    rightShoulder.enableVoltageCompensation(12.0);

    rightShoulder.setSoftLimit(SoftLimitDirection.kForward, 40);
    rightShoulder.setSoftLimit(SoftLimitDirection.kReverse, -40);

    leftShoulder.follow(rightShoulder, true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    
    shoulderEnc = rightShoulder.getEncoder();
    shoulderEnc.setPosition(0);
    shoulderPos = shoulderTab.add("ShoulderPos", getShoulderPos()).getEntry();

    shoulderPID = rightShoulder.getPIDController();
    shoulderPID.setP(ArmConstants.kShoulderP, 0);
    shoulderP = shoulderTab.add("ShoulderP", shoulderPID.getP(0)).getEntry();
    shoulderPID.setI(ArmConstants.kShoulderI, 0);
    shoulderI = shoulderTab.add("ShoulderI", shoulderPID.getI(0)).getEntry();
    shoulderPID.setD(ArmConstants.kShoulderD, 0);
    shoulderD = shoulderTab.add("ShoulderD", shoulderPID.getD(0)).getEntry();
    shoulderPID.setFF(ArmConstants.kShoulderFF, 0);
    shoulderFF = shoulderTab.add("ShoulderFF", shoulderPID.getFF(0)).getEntry();

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();

    shoulderSetpoint = shoulderTab.add("ShoulderSetpoint", 0).getEntry();
  }

  public void moveShoulderToPosition(double degrees, double arbFF) {
    shoulderPID.setReference(degrees, ControlType.kPosition, 0, arbFF);
  }

  public void moveShoulderToSetpoint() {
    shoulderPID.setReference(shoulderSetpoint.getDouble(0), ControlType.kPosition);
  }

  public void moveShoudler(double speed) {
    rightShoulder.set(speed);
  }

  public double getShoulderPos() {
    // 36.5
    // -37.8
    return shoulderEnc.getPosition() * 2.423;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderPos.setDouble(getShoulderPos());
    double tempP = shoulderP.getDouble(shoulderPID.getP(0));
    if (shoulderPID.getP(0) != tempP) {
      shoulderPID.setP(tempP, 0);
    }
    double tempI = shoulderI.getDouble(shoulderPID.getI(0));
    if (shoulderPID.getI(0) != tempI) {
      shoulderPID.setI(tempI, 0);
    }
    double tempD = shoulderD.getDouble(shoulderPID.getD(0));
    if (shoulderPID.getD(0) != tempD) {
      shoulderPID.setD(tempD, 0);
    }
    double tempFF = shoulderFF.getDouble(shoulderPID.getFF(0));
    if (shoulderPID.getFF(0) != tempFF) {
      shoulderPID.setFF(tempFF, 0);
    }
  }
}
