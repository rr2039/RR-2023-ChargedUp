// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
  int half_leds = m_ledBuffer.getLength()/2;
  int m_rainbowFirstPixelHue = 0;
  // Cone new Color(242, 203, 5)
  // Cube Color.kPurple
  Color gamePiece = Color.kPurple;

  AnalogInput soundSensor = new AnalogInput(0);

  /** Creates a new LEDController. */
  public LEDController() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    //soundSensor.setOversampleBits(4);
    //soundSensor.setAverageBits(4);
  }

  public void setGamePiece(int piece) {
    gamePiece = (piece == 1 ? Color.kPurple : Color.kYellow);
  }

  private void halvsies() {
    for (int i = 0; i < half_leds; i++) {
      m_ledBuffer.setLED(i, Color.kRed);
    }
    for (int i = half_leds; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kBlue);
    }
  }

  private void setGamePieceAndAlliance() {
    Color allianceColor = DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed;
    for (int i = 0; i < half_leds/2 - 11; i++) {
      m_ledBuffer.setLED(i, allianceColor); 
    }
    for (int i = half_leds/2 - 11; i < half_leds/2 + 11; i++) {
      m_ledBuffer.setLED(i, gamePiece);
    }
    for (int i = half_leds/2 + 11; i < half_leds + half_leds/2 - 10; i++) {
      m_ledBuffer.setLED(i, allianceColor); 
    }
    for (int i = half_leds + half_leds/2 - 10; i < half_leds + half_leds/2 + 12; i++){
      m_ledBuffer.setLED(i, gamePiece);
    }
    for (int i = half_leds + half_leds/2 + 12; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, allianceColor);
    }
  }

  private void rainbow() {
    // For every pixel
    for (int i = 0; i < half_leds; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength()/2)) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    for (int i = half_leds; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength()/2)) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
    // Fill the buffer with a rainbow
    //halvsies();
    //rainbow();
    setGamePieceAndAlliance();
    // Set the LEDs
    m_led.setData(m_ledBuffer);
    SmartDashboard.putNumber("SoundLevel", soundSensor.getValue());
  }
}
