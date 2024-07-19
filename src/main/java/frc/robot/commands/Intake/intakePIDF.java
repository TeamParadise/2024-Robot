// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class intakePIDF extends Command {
  /** Creates a new intakeController. */

  private final SparkPIDController intakePIDController;
  
  double rpm;
  public intakePIDF(double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeSubsystem);
    
    intakePIDController = RobotContainer.m_intakeSubsystem.intakeMotor.getPIDController();
    // SmartDashboard.putNumber("intake Setpoint", 0);
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rpm = SmartDashboard.getNumber("intake Setpoint", 0);
    // SmartDashboard.putNumber("intake speed", RobotContainer.m_intakeSubsystem.intakeMotor.getEncoder().getVelocity());
    intakePIDController.setReference(-rpm, CANSparkBase.ControlType.kVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
