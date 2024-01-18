// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED ledStrip = new AddressableLED(LEDConstants.ledPort);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);

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

  public void halfAndHalf(Color color1, Color color2) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setLED(i, color1);
      } else {
        ledBuffer.setLED(i, color2);
      }
    }
    ledStrip.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
