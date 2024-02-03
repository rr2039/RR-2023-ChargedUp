// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {

  CANSparkMax rightExtendyGirl;
  CANSparkMax leftExtendyGirl; 
  CANSparkMax rightExtendyBoy;
  CANSparkMax leftExtendyBoy; 

  RelativeEncoder extendyGirlEnc;
  RelativeEncoder extendyBoyEnc;

  SparkPIDController extendyGirlPID;
  SparkPIDController extendyBoyPID;

  double extendyGirlCurSetpoint = ArmConstants.kExtendyGirlStartPoint;
  double extendyBoyCurSetpoint = ArmConstants.kExtendyBoyStartPoint;

  // Shuffleboard Tab for Shoulder Updates
  ShuffleboardTab armExtensionTab = Shuffleboard.getTab("Arm");
  GenericEntry extendyBoyPos;
  GenericEntry extendyGirlPos;

  /** Creates a new ArmExtensionSubsystem. */
  public ArmExtensionSubsystem() {

    rightExtendyGirl = new CANSparkMax(ArmConstants.kRightExtendyGirlCanId, MotorType.kBrushless);
    leftExtendyGirl = new CANSparkMax(ArmConstants.kLeftExtendyGirlCanId, MotorType.kBrushless);
    rightExtendyBoy = new CANSparkMax(ArmConstants.kRightExtendyBoyCanId, MotorType.kBrushless);
    leftExtendyBoy = new CANSparkMax(ArmConstants.kLeftExtendyBoyCanId, MotorType.kBrushless);

    leftExtendyGirl.restoreFactoryDefaults();
    rightExtendyGirl.restoreFactoryDefaults();
    leftExtendyBoy.restoreFactoryDefaults();
    rightExtendyBoy.restoreFactoryDefaults();

    rightExtendyGirl.setInverted(false);

    //rightExtendyGirl.setClosedLoopRampRate(3.5);
    //rightExtendyBoy.setClosedLoopRampRate(3.5);

    //rightExtendyBoy.setSmartCurrentLimit(10, 40);
    //rightExtendyGirl.setSmartCurrentLimit(10, 40);

    leftExtendyGirl.follow(rightExtendyGirl, true);
    leftExtendyBoy.follow(rightExtendyBoy, true);

    rightExtendyGirl.setIdleMode(IdleMode.kBrake);
    leftExtendyGirl.setIdleMode(IdleMode.kBrake);
    rightExtendyBoy.setIdleMode(IdleMode.kBrake);
    leftExtendyBoy.setIdleMode(IdleMode.kBrake);

    extendyGirlEnc = rightExtendyGirl.getEncoder();
    extendyBoyEnc = rightExtendyBoy.getEncoder();
    // Old Factor 0.08858
    extendyGirlEnc.setPositionConversionFactor(0.111666);
    extendyBoyEnc.setPositionConversionFactor(0.111666);
    extendyBoyEnc.setPosition(ArmConstants.kExtendyBoyStartPoint);
    extendyGirlEnc.setPosition(ArmConstants.kExtendyGirlStartPoint);
    extendyBoyPos = armExtensionTab.add("Extendy Boy Pos", getExtendyBoyPos()).getEntry();
    extendyGirlPos = armExtensionTab.add("Extendy Girl Pos", getExtendyGirlPos()).getEntry();

    extendyGirlPID = rightExtendyGirl.getPIDController();
    extendyGirlPID.setP(ArmConstants.kGirlsP, 0);
    extendyGirlPID.setI(ArmConstants.kGirlsI, 0);
    extendyGirlPID.setD(ArmConstants.kGirlsD, 0);
    extendyGirlPID.setFF(ArmConstants.kGirlsFF, 0);
    extendyBoyPID = rightExtendyBoy.getPIDController();
    extendyBoyPID.setP(ArmConstants.kBoysP, 0);
    extendyBoyPID.setI(ArmConstants.kBoysI, 0);
    extendyBoyPID.setD(ArmConstants.kBoysD, 0);
    extendyBoyPID.setFF(ArmConstants.kBoysFF, 0);

    rightExtendyGirl.burnFlash();
    leftExtendyGirl.burnFlash();
    rightExtendyBoy.burnFlash();
    leftExtendyBoy.burnFlash();

  }

  public double getExtendyBoyCurSetpoint() {
    return extendyBoyCurSetpoint;
  }

  public double getExtendyGirlCurSetpoint() {
    return extendyGirlCurSetpoint;
  }

  public void setExtendyBoyCurSetpoint(double setpoint) {
    if (setpoint >= -1) {
      extendyBoyCurSetpoint = setpoint;
    }
  }

  public void setExtendyGirlCurSetpoint(double setpoint) {
    if (setpoint >= -1) {
      extendyGirlCurSetpoint = setpoint;
    }
  }

  public void moveExtendyGirlsToPos(double pos, double arbFF) {
    if (pos >= -1) {
      extendyGirlPID.setReference(pos, ControlType.kPosition, 0, arbFF);
    }
  }

  public void moveExtendyBoysToPos(double pos, double arbFF) {
    if (pos >= -1) {
      extendyBoyPID.setReference(pos, ControlType.kPosition, 0, arbFF);
    }
  }

  public void moveExtendyGirls(double speed) {
    rightExtendyGirl.set(speed);
  }
  
  public void moveExtendyBoys(double speed) {
    rightExtendyBoy.set(speed);
  }

  public void moveExtendors(double speed) {
    rightExtendyGirl.set(speed);
    rightExtendyBoy.set(speed);
  }

  public double getExtendyBoyPos() {
    return extendyBoyEnc.getPosition();
  }

  public double getExtendyGirlPos() {
    return extendyGirlEnc.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    extendyBoyPos.setDouble(getExtendyBoyPos());
    extendyGirlPos.setDouble(getExtendyGirlPos());
  }
}
