// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.MotorConstants;         


public class PrimerSubsystem extends SubsystemBase {
  /** Creates a new PrimerSubsystem. */

  CANSparkMax primerNeo = new CANSparkMax(MotorConstants.leftPrimerMotorID, MotorType.kBrushless);

  public PrimerSubsystem() {
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void configMotors(){
    primerNeo.setSmartCurrentLimit(25, 20);
    primerNeo.setOpenLoopRampRate(0.15);
    primerNeo.setInverted(false);
  }

  public void runMotors(double speed){
    primerNeo.set(speed);
  }

  public void stopMotors() {
    primerNeo.stopMotor();
  }
}
