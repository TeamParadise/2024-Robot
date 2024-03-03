// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class armAutoShoot extends Command {
  /** Creates a new armAutoShoot. */
  
  private final SparkPIDController leftPIDController, rightPIDController;

  double angle;
  double speed;

  public armAutoShoot(double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ArmSubsystem);
    addRequirements(RobotContainer.m_shooterSubsystem);

    this.angle = angle;
    this.speed = speed;

    leftPIDController = RobotContainer.m_shooterSubsystem.leftShooter.getPIDController();
    rightPIDController = RobotContainer.m_shooterSubsystem.rightShooter.getPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
