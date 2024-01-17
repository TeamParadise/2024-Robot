// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import swervelib.encoders.CanAndCoderSwerve;      
import frc.robot.Constants.MotorConstants;         


public class PrimerSubsystem extends SubsystemBase {
  /** Creates a new PrimerSubsystem. */

  CANSparkMax PrimerNeo = new CANSparkMax(MotorConstants.leftPrimerMotorID, MotorType.kBrushless);

  public PrimerSubsystem() {
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void configMotors(){
    PrimerNeo.setSmartCurrentLimit(25, 20);
    PrimerNeo.setOpenLoopRampRate(0.15);
    PrimerNeo.setInverted(false);
  }

  public void runMotors(double speed){
    PrimerNeo.set(speed);
  }
}
