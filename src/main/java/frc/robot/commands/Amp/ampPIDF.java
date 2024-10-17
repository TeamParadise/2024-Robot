// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ampPIDF extends Command {
  /** Creates a new ampPIDF. */
  private final SparkPIDController ampPIDController;
  
  double rpm;
  public ampPIDF(double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ampSubsystem);
    ampPIDController = RobotContainer.m_ampSubsystem.ampMotor.getPIDController();
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampPIDController.setReference(rpm, CANSparkBase.ControlType.kVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ampSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
