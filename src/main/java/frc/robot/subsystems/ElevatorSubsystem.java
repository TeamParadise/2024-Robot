// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX leftElevatorMotor, rightElevatorMotor;

  public ElevatorSubsystem() {
    configMotors();
  }

  public void configMotors(){
    leftElevatorMotor = new TalonFX(Constants.MotorConstants.leftElevatorMotorID, "rhino");
    rightElevatorMotor = new TalonFX(Constants.MotorConstants.rightElevatorMotorID, "rhino");
    resetEncoder();
  }

  public void setSpeed(double speed){
    if (RobotContainer.m_intakeSubsystem.getArmPos() < 130 && RobotContainer.m_intakeSubsystem.getArmPos() > 20){
      leftElevatorMotor.set(speed);
      rightElevatorMotor.set(speed);
      // System.out.println(speed + ", " + getEncoder());
      // System.out.println(RobotContainer.m_ArmSubsystem.getArmPos());
    }
    else {
      System.out.println("stopping");
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
    }
    System.out.println(speed + ", " + getEncoder());
  }

  public double getEncoder(){
    return rightElevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder(){
    leftElevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
