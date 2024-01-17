// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetArmPosition extends Command {
  PIDController armPID;
  double angle;
  /** Creates a new SetPosition. */
  public SetArmPosition(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
    this.angle = angle;
    armPID = new PIDController(0, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.armSubsystem.setMotors(armPID.calculate(RobotContainer.armSubsystem.getArmEncoder(), angle));

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
