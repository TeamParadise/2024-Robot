// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }
  
  public void setAll(Color color) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color);
      }
      ledStrip.setData(ledBuffer);
  }

  public void setAll(int hue, int saturation, int value) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, saturation, value);
    }
    ledStrip.setData(ledBuffer);
  }

  public void setIndividual(int number, Color color) {
    ledBuffer.setLED(number, color);
    ledStrip.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
