// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PrimerSubsystem extends SubsystemBase {
  /** Creates a new PrimerSubsystem. */
  CANSparkMax leftPrimer, rightPrimer;
  public PrimerSubsystem() {
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void configMotors(){
    leftPrimer = new CANSparkMax(Constants.MotorConstants.leftPrimerMotorID, MotorType.kBrushless);
    rightPrimer = new CANSparkMax(Constants.MotorConstants.rightPrimerMotorID, MotorType.kBrushless);

    leftPrimer.setSmartCurrentLimit(60);
    rightPrimer.setSmartCurrentLimit(60);

    leftPrimer.setIdleMode(IdleMode.kCoast);
    rightPrimer.setIdleMode(IdleMode.kCoast);
  }

  public void setSpeed(double speed){
    leftPrimer.set(-speed);
    rightPrimer.set(speed);
  }

}
