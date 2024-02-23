// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  TalonFX leftArmMotor, rightArmMotor;
  // CANSparkMax sparkMax = new CANSparkMax(7, MotorType.kBrushless);
  // AbsoluteEncoder absEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
  public ArmSubsystem() {
    configMotors();
  }

  private void configMotors() {
    leftArmMotor = new TalonFX(Constants.MotorConstants.leftArmMotorID, "rhino");
    rightArmMotor = new TalonFX(Constants.MotorConstants.rightArmMotorID, "rhino");
  }

  public void setSpeed(double speed){
    leftArmMotor.set(speed);
    rightArmMotor.set(-speed);

  }

  public void setVoltage(double voltage) {
    leftArmMotor.setVoltage(voltage);
    rightArmMotor.setVoltage(voltage);
  }
 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
