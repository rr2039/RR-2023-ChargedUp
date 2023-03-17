// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  SparkMaxPIDController extendyGirlPID;
  SparkMaxPIDController extendyBoyPID;

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

    leftExtendyGirl.follow(rightExtendyGirl, true);
    leftExtendyBoy.follow(rightExtendyBoy, true);

    rightExtendyGirl.setIdleMode(IdleMode.kBrake);
    leftExtendyGirl.setIdleMode(IdleMode.kBrake);
    rightExtendyBoy.setIdleMode(IdleMode.kBrake);
    leftExtendyBoy.setIdleMode(IdleMode.kBrake);

    extendyGirlEnc = rightExtendyGirl.getEncoder();
    extendyBoyEnc = rightExtendyBoy.getEncoder();
    extendyBoyEnc.setPosition(0);
    extendyGirlEnc.setPosition(0);
    extendyBoyPos = armExtensionTab.add("Extendy Boy Pos", getExtendyBoyPos()).getEntry();
    extendyGirlPos = armExtensionTab.add("Extendy Girl Pos", getExtendyGirlPos()).getEntry();

    extendyGirlPID = rightExtendyGirl.getPIDController();
    extendyBoyPID = rightExtendyBoy.getPIDController();

    rightExtendyGirl.burnFlash();
    leftExtendyGirl.burnFlash();
    rightExtendyBoy.burnFlash();
    leftExtendyBoy.burnFlash();

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
    return extendyGirlEnc.getPosition() * -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    extendyBoyPos.setDouble(getExtendyBoyPos());
    extendyGirlPos.setDouble(getExtendyGirlPos());
  }
}
