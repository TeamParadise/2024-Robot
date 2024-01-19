// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class primer extends SubsystemBase {
  /** Creates a new primer. */
  public primer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  DigitalInput input = new DigitalInput(0);

  

  public boolean getIRSensor(){
    return input.get();
  } 

}
