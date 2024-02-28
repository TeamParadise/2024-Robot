// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class alignNote extends Command {
  PhotonPipelineResult visionResult;
  PIDController turnController;
  double tx;
  
  /** Creates a new allignNote. */
  public alignNote() {
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(0.1, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionResult = RobotContainer.vision.getLatestIntakeResult();
    if (visionResult.hasTargets()) {
      tx = visionResult.getBestTarget().getYaw();
      if (Math.abs(tx) > 1) {
        RobotContainer.drivetrain.applyRequest(() -> RobotContainer.robotDrive.withRotationalRate(turnController.calculate(tx, 0)));
      } else {
        RobotContainer.drivetrain.applyRequest(() -> RobotContainer.robotDrive.withVelocityY(-0.1)).withTimeout(1);
      }
    }
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
