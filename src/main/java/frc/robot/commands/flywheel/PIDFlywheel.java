// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PIDFlywheel extends Command {
  /** Creates a new PIDFlywheel. */
  PIDController leftController, rightController;
  double rightSpeed, leftSpeed, rpm;

  public PIDFlywheel(double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.flywheelSubsystem);
    leftController = new PIDController(0, 0, 0);
    rightController = new PIDController(1, 0, 0);
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftController.reset();
    rightController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftSpeed = leftController.calculate(RobotContainer.flywheelSubsystem.getLeftEncoder(), rpm);
    rightSpeed = leftController.calculate(RobotContainer.flywheelSubsystem.getRightEncoder(), rpm);
    RobotContainer.flywheelSubsystem.runMotors(leftSpeed, rightSpeed);
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
