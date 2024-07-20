// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public CANSparkMax leftShooter;
  public CANSparkMax rightShooter;
  public SparkPIDController leftPIDController;
  public SparkPIDController rightPIDController;
  public double speed = 0;
  AbsoluteEncoder absEncoder;

  public ShooterSubsystem() {
    configMotors();
  }

  public void configMotors(){
    leftShooter = new CANSparkMax(Constants.MotorConstants.leftShooterMotorID, MotorType.kBrushless);
    rightShooter = new CANSparkMax(Constants.MotorConstants.rightShooterMotorID, MotorType.kBrushless);
    leftPIDController = leftShooter.getPIDController();
    rightPIDController = rightShooter.getPIDController();
    // absEncoder.setPositionConversionFactor(2);
    // absEncoder.setVelocityConversionFactor(2);

    leftPIDController.setP(ShooterConstants.kLeftP);
    leftPIDController.setI(ShooterConstants.kLeftI);
    leftPIDController.setD(ShooterConstants.kLeftD);
    leftPIDController.setIZone(ShooterConstants.kLeftIz);
    leftPIDController.setFF(ShooterConstants.kLeftFF);
    leftPIDController.setOutputRange(ShooterConstants.kLeftMin, ShooterConstants.kLeftMax);

    rightPIDController.setP(ShooterConstants.kRightP);
    rightPIDController.setI(ShooterConstants.kRightI);
    rightPIDController.setD(ShooterConstants.kRightD);
    rightPIDController.setIZone(ShooterConstants.kRightIz);
    rightPIDController.setFF(ShooterConstants.kRightFF);
    rightPIDController.setOutputRange(ShooterConstants.kRightMin, ShooterConstants.kRightMax);

    leftShooter.setSmartCurrentLimit(30);
    rightShooter.setSmartCurrentLimit(30);
  }

  public void setSpeed(double speed){
    leftShooter.set(speed);
    rightShooter.set(-speed);
  }

  public void setVoltage(double leftMotorVolts, double rightMotorVolts){
    leftShooter.setVoltage(leftMotorVolts);
    rightShooter.setVoltage(rightMotorVolts);
  }

  public double getSpeed(){
    return speed;
  }
  
  public void setSpeedValue(double speed){
    this.speed = speed;
  }

  public double calculateOptimalVelocity(double vF, double height, double xRobotVelocity, double armAngle) {
    double rpm = ((30 *
    (Math.sqrt(2 * - 9.81 * height + Math.pow(vF,2))
    - xRobotVelocity * Math.tan(armAngle)))
    / (Math.PI * Constants.PhysicalConstants.shooterWheelRadiusMeters));
    return rpm;
  }

  public double getAverageVelocity() {
    return (leftShooter.getEncoder().getVelocity() + rightShooter.getEncoder().getVelocity()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 
}
