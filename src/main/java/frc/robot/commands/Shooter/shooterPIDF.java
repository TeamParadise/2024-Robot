// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class shooterPIDF extends Command {
  private final SparkPIDController leftPIDController, rightPIDController;
  private double setpoint;

  /** Creates a new shooterPIDF. */
  public shooterPIDF(double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooterSubsystem);
    leftPIDController = RobotContainer.m_shooterSubsystem.leftShooter.getPIDController();
    rightPIDController = RobotContainer.m_shooterSubsystem.rightShooter.getPIDController();
    this.setpoint = velocity;
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Speed", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // setpoint = SmartDashboard.getNumber("Speed", 0);
    leftPIDController.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
    rightPIDController.setReference(-setpoint, CANSparkBase.ControlType.kVelocity);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
