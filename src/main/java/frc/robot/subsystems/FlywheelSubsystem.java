// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.MotorConstants;         


public class FlywheelSubsystem extends SubsystemBase {

  CANSparkMax leftMotor = new CANSparkMax(MotorConstants.leftFlywheelMotorID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(MotorConstants.rightFlywheelMotorID, MotorType.kBrushless);

  public FlywheelSubsystem() {
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void configMotors(){
    leftMotor.setSmartCurrentLimit(25, 20);
    rightMotor.setSmartCurrentLimit(25, 20);

    leftMotor.setOpenLoopRampRate(0.15);
    rightMotor.setOpenLoopRampRate(0.15);

    rightMotor.setInverted(false);
    leftMotor.setInverted(false);

    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
  }
  
  public void stopFlywheel(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void runMotors(double speed){
    leftMotor.set(Math.abs(speed));
    rightMotor.set(Math.abs(speed));
  }

  public void runMotors(double leftSpeed, double rightSpeed){
    leftMotor.set(Math.abs(leftSpeed));
    rightMotor.set((rightSpeed));
  }

  public double getLeftEncoder(){
    return leftMotor.getEncoder().getVelocity();
  }

  public double getRightEncoder(){
    return rightMotor.getEncoder().getVelocity();
  }
}
