// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
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
    return Math.sqrt(Math.pow(16.5 - drivetrain.getState().Pose.getX() , 2) + Math.pow(5.475 - drivetrain.getState().Pose.getY(), 2));
  }
  
  //return optimal angle for the arm (for blue side)
  public double findOptimalAngle(){
    double speakerHeight = 2; //meters
    // double armHeightOffset = 0;
    // double armPositionOffset = 0;  
    double distance = Math.sqrt(Math.pow(16.5 - drivetrain.getState().Pose.getX() , 2) + Math.pow(5.475 - drivetrain.getState().Pose.getY(), 2));
    double angle = Math.atan(speakerHeight / distance);
    return Units.radiansToDegrees(angle);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
