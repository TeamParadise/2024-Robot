// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder ;
public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  TalonFX leftArmMotor, rightArmMotor;
  DutyCycleEncoder  boreEncoder;
  public ArmSubsystem() {
    leftArmMotor = new TalonFX(Constants.MotorConstants.leftArmMotorID);
    rightArmMotor = new TalonFX(Constants.MotorConstants.rightArmMotorID);
    boreEncoder = new DutyCycleEncoder(2); 

    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configMotors(){
    leftArmMotor.setNeutralMode(NeutralModeValue.Brake);
    rightArmMotor.setNeutralMode(NeutralModeValue.Brake);

    leftArmMotor.setInverted(false);
    rightArmMotor.setInverted(false);
  }

  public void setMotors(double speed){
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public double getArmEncoder(){
    return boreEncoder.getAbsolutePosition();
  }
}
