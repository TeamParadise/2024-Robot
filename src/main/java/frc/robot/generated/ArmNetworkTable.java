// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.TreeMap;

public class ArmNetworkTable <Key extends Number, Value extends Number> extends SubsystemBase {
  /** Creates a new ArmNetworkTable. */
  public ArmNetworkTable(){
    zfinal TreeMap<Key, Value> m_map = new TreeMap<>();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
