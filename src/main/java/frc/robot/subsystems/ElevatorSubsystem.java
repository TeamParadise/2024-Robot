// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.MotorConstants;         


public class ElevatorSubsystem extends SubsystemBase {

  CANSparkMax leftMotor = new CANSparkMax(MotorConstants.leftElevatorMotorID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(MotorConstants.rightElevatorMotorID, MotorType.kBrushless);

  public ElevatorSubsystem() {
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
  }

  public void runMotors(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void runMotors(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }
  
}
