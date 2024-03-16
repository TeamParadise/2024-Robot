// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class elevatorController extends Command {
  /** Creates a new elevatorSubsystem. */
  double setpoint;
  PIDController elevatorController;
  public elevatorController(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies
    
    addRequirements(RobotContainer.m_ElevatorSubsystem);
    elevatorController = new PIDController(0.5, 0, .00025);
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorController.reset();
    elevatorController.setSetpoint(setpoint);
    SmartDashboard.putNumber("elevator setpoint", setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // setpoint = SmartDashboard.getNumber("elevator setpoint", 0);
    setpoint = MathUtil.clamp(setpoint, 0, 52.5);
    elevatorController.setSetpoint(setpoint);
    double output = elevatorController.calculate(RobotContainer.m_ElevatorSubsystem.getEncoder());
    RobotContainer.m_ElevatorSubsystem.setSpeed(MathUtil.clamp(output, -0.4, 0.4));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ElevatorSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
