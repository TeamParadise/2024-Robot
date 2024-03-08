// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intakeMotor;
  AbsoluteEncoder absEncoder;
  public DigitalInput intakeBeamBreaker;
  public IntakeSubsystem() {
    intakeBeamBreaker = new DigitalInput(0);
    configMotors();
  }

  private void configMotors() {
    intakeMotor = new CANSparkMax(Constants.MotorConstants.intakeMotorID, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(60);
    absEncoder = intakeMotor.getAbsoluteEncoder(Type.kDutyCycle);

  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public double getArmPosition(){
    return absEncoder.getPosition();
  }



  public boolean getIntakeBeamBreaker() {
    return !intakeBeamBreaker.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake beam breaker state", getIntakeBeamBreaker());
  }
}
