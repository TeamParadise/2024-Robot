// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  CANSparkMax leftShooter, rightShooter;
  public double speed = 0;
  RelativeEncoder leftEncoder, rightEncoder;
  AbsoluteEncoder absEncoder;

  public ShooterSubsystem() {
    configMotors();
  }

  public void configMotors(){
    leftShooter = new CANSparkMax(Constants.MotorConstants.leftShooterMotorID, MotorType.kBrushless);
    rightShooter = new CANSparkMax(Constants.MotorConstants.rightShooterMotorID, MotorType.kBrushless);
    leftEncoder = leftShooter.getEncoder();
    rightEncoder = rightShooter.getEncoder();

    absEncoder = leftShooter.getAbsoluteEncoder(Type.kDutyCycle);
    // absEncoder.setPositionConversionFactor(2);
    // absEncoder.setVelocityConversionFactor(2);

    leftShooter.setSmartCurrentLimit(50);
    rightShooter.setSmartCurrentLimit(50);
  }

  public void setSpeed(double speed){
    System.out.println(speed);
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void setVoltage(double leftMotorVolts, double rightMotorVolts){
    leftShooter.setVoltage(leftMotorVolts);
    rightShooter.setVoltage(rightMotorVolts);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(rightEncoder.getPosition() / 1);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / 1);
    inputs.appliedVolts = leftShooter.getAppliedOutput() * leftShooter.getBusVoltage();
    inputs.currentAmps = new double[] {leftShooter.getOutputCurrent(), rightShooter.getOutputCurrent()};
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 
}
