// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.TreeMap;

public class ArmLUT <Key extends Number, Value extends Number> extends SubsystemBase {
  
  //Create empty LUT
  final TreeMap<Key, Value> table = new TreeMap<>();
  
  /** Creates a new ArmNetworkTable. */
  public ArmLUT() {
  }

  public void put(Key key, Value value) {
    table.put(key, value);
  }

  public Double get(Key key){
    Value value = table.get(key);
    
    if (value == null) {
      Key upperKey = table.ceilingKey(key);
      Key lowerKey = table.floorKey(key);

      if (upperKey == null && lowerKey == null) {
        return null;
      }
      if (upperKey == null) {
        return table.get(lowerKey).doubleValue();
      }
      if (lowerKey == null) {
        return table.get(upperKey).doubleValue();
      }
      
      Value lowerValue = table.get(lowerKey);
      Value upperValue = table.get(upperKey);

      return interpolate(lowerValue, upperValue, lowerKey, upperKey, key);
    } else {
      return value.doubleValue();
    }    
  }

  public double interpolate (Value lowVal, Value upVal, Key lowKey, Key upKey, Key key) {
    //Linear Interpolations
    return ((key.doubleValue() - lowKey.doubleValue())/(upKey.doubleValue() - lowKey.doubleValue()) * (upVal.doubleValue() - lowVal.doubleValue()) + lowVal.doubleValue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
