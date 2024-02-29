// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SpeedTune extends InstantCommand {
  /** Creates a new SpeedTune. */
  double speed;
  double output;
  public SpeedTune(double tune) {
    // Use addRequirements() here to declare subsystem dependencies.
    speed = tune;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    output = RobotContainer.m_shooterSubsystem.getSpeed() + speed;
    RobotContainer.m_shooterSubsystem.setSpeedValue(output);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
