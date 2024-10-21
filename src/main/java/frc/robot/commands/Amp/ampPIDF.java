// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmpConstants;
import frc.robot.RobotContainer;

public class ampPIDF extends Command {
  /** Creates a new ampPIDF. */
  private final SparkPIDController ampPIDController;
  
  double position;
  public ampPIDF(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ampSubsystem);
    ampPIDController = RobotContainer.m_ampSubsystem.ampMotor.getPIDController();
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampPIDController.setReference(position, CANSparkBase.ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampPIDController.setReference(AmpConstants.kPosDown, CANSparkBase.ControlType.kPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
