// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AmpConstants;

public class AmpSubsystem extends SubsystemBase {
  public CANSparkMax ampMotor;
  public SparkPIDController ampPIDController;
 
  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    configMotors();
  }

  private void configMotors() {
    ampMotor = new CANSparkMax(Constants.MotorConstants.ampMotorID, MotorType.kBrushless);
    ampMotor.setSmartCurrentLimit(25);

    ampPIDController = ampMotor.getPIDController();
    ampPIDController.setOutputRange(AmpConstants.kMin, AmpConstants.kMax);
    ampPIDController.setP(AmpConstants.kP);
    ampPIDController.setI(AmpConstants.kI);
    ampPIDController.setD(AmpConstants.kD);
    ampPIDController.setIZone(AmpConstants.kIz);
    ampPIDController.setFF(AmpConstants.kFF);

    // Tuning, delete later
    SmartDashboard.putNumber("Amp kP", AmpConstants.kP);
    SmartDashboard.putNumber("Amp kI", AmpConstants.kI);
    SmartDashboard.putNumber("Amp kD", AmpConstants.kD);
    SmartDashboard.putNumber("Amp kIz", AmpConstants.kIz);
    SmartDashboard.putNumber("Amp kFF", AmpConstants.kFF);
    SmartDashboard.putNumber("Amp kMin", AmpConstants.kMin);
    SmartDashboard.putNumber("Amp kMax", AmpConstants.kMax);
  }

  public void setSpeed(double speed) {
    ampMotor.set(speed);
  }

  // Tuning, delete later
  public void setPID() {
    ampPIDController.setOutputRange(SmartDashboard.getNumber("Amp kMin", AmpConstants.kMin), SmartDashboard.getNumber("Amp kMax", AmpConstants.kMax));
    ampPIDController.setP(SmartDashboard.getNumber("Amp kP", AmpConstants.kP));
    ampPIDController.setI(SmartDashboard.getNumber("Amp kI", AmpConstants.kI));
    ampPIDController.setD(SmartDashboard.getNumber("Amp kD", AmpConstants.kD));
    ampPIDController.setIZone(SmartDashboard.getNumber("Amp kIz", AmpConstants.kIz));
    ampPIDController.setFF(SmartDashboard.getNumber("Amp kFF", AmpConstants.kFF));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
