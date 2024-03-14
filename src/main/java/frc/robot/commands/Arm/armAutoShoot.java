// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class armAutoShoot extends Command {
  /** Creates a new armAutoShoot. */
  
  private final SparkPIDController leftPIDController, rightPIDController;


  double setpoint, output, positionDegrees, positionRadians, velocity;

  private final double 
    kp = .55, 
    ki = 0, 
    kd = 0,
    maxAccel = .5,
    maxVelo = 1;
  
  double MaxSpeed = 6;

  /*feedforwardMax is the max volts that need to be supplied to match gravity (arm at 0 degrees)*/
  public double feedforwardMax = 0;
  PIDController armController = new PIDController(kp, ki, kd);

  public armAutoShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ArmSubsystem);
    addRequirements(RobotContainer.m_shooterSubsystem);

    leftPIDController = RobotContainer.m_shooterSubsystem.leftShooter.getPIDController();
    rightPIDController = RobotContainer.m_shooterSubsystem.rightShooter.getPIDController();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Shooter flywheels
    double speed = Robot.m_ArmLUTRPM.get(RobotContainer.m_ArmSubsystem.getDistance());
    leftPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
    rightPIDController.setReference(-speed, CANSparkBase.ControlType.kVelocity);

    //Arm angle
    double angle = Robot.m_ArmLUTAngle.get(RobotContainer.m_ArmSubsystem.getDistance());
    angle = MathUtil.clamp(angle, 0, 60);
    positionDegrees = RobotContainer.m_intakeSubsystem.getArmPosition();
    positionRadians = Math.toRadians(Math.toRadians(positionDegrees));
    output = armController.calculate(positionDegrees, angle) + feedforwardMax * Math.cos(positionRadians);
    RobotContainer.m_ArmSubsystem.setVoltage(MathUtil.clamp(output, -7.5, 7.5));

    //Auto heading
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ArmSubsystem.setVoltage(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
