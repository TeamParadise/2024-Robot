// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  TalonFX leftArmMotor, rightArmMotor;
  // CANSparkMax sparkMax = new CANSparkMax(7, MotorType.kBrushless);
  // AbsoluteEncoder absEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
  public ArmSubsystem() {
    configMotors();
  }

  private void configMotors() {
    leftArmMotor = new TalonFX(Constants.MotorConstants.leftArmMotorID, "rhino");
    rightArmMotor = new TalonFX(Constants.MotorConstants.rightArmMotorID, "rhino");
  }

  public void setSpeed(double speed){
    leftArmMotor.set(speed);
    rightArmMotor.set(-speed);

  }

  public void setVoltage(double voltage) {
    leftArmMotor.setVoltage(voltage);
    rightArmMotor.setVoltage(-voltage);
  }

  public double getDistance(){
    //Red
    return Math.sqrt(Math.pow(16.5 - drivetrain.getState().Pose.getX() , 2) + Math.pow(5.475 - drivetrain.getState().Pose.getY(), 2));

    //Blue
    // return Math.sqrt(Math.pow(drivetrain.getState().Pose.getX() , 2) + Math.pow(5.475 - drivetrain.getState().Pose.getY(), 2));

  }
  
  //return optimal angle for the arm (for blue side)
  public double findOptimalAngle(){
    double distance = getDistance();
    return Robot.m_ArmLUTAngle.get(distance);

    /*double speakerHeight = Units.inchesToMeters(82.5 + Math.pow(distance, 2) / 100) - Units.inchesToMeters(20); //meters
    // double armHeightOffset = 0;
    // double armPositionOffset = 0;  
    double distance = Math.sqrt(Math.pow(16.5 - drivetrain.getState().Pose.getX() , 2) + Math.pow(5.475 - drivetrain.getState().Pose.getY(), 2));
    double angle = Math.atan(speakerHeight / distance);
    // double v0 = 21.2;
    // double gravity = -9.81;
    //double angle = Math.sqrt(Math.tan(speakerHeight/distance));
    /*double angle = Math.atan(
      Math.pow(v0, 2) + Math.sqrt(Math.pow(v0, 4)
      - gravity * (gravity * Math.pow(distance, 2) + 2 * speakerHeight * Math.pow(v0, 2)))
      / (gravity * distance));
    // double angle = 1.736 * Math.pow(distance, 2) - 16.645 * distance + 65.301;
    SmartDashboard.putNumber("horizontalDistance", distance);
    SmartDashboard.putNumber("outputangle", Units.radiansToDegrees(angle));

    return Units.radiansToDegrees(angle);*/
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
