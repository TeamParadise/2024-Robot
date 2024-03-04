// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class alignNoteDrive extends Command {
  PhotonPipelineResult visionResult;
  PIDController turnController;
  double tx, lastTimeDetected;
  
  /** Creates a new allignNote. */
  public alignNoteDrive() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(0.12, 0.0, 0.03);
    turnController.reset();
    SmartDashboard.putNumber("kp Auto Align", 0.1);
    SmartDashboard.putNumber("kd Auto Align", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnController.setP(SmartDashboard.getNumber("kp Auto Align", 0.12));
    turnController.setD(SmartDashboard.getNumber("kd Auto Align", 0.03));
    visionResult = RobotContainer.vision.intakeCamera.getLatestResult();
    if (visionResult.hasTargets()) {
      lastTimeDetected = 0;
      RobotContainer.drivetrain.setControl(RobotContainer.robotDrive.withRotationalRate(turnController.calculate(visionResult.getBestTarget().getYaw(), 0)).withVelocityX(-4));
    } else if (lastTimeDetected < 0.5) {
      lastTimeDetected = RobotController.getFPGATime() - lastTimeDetected;
    } else {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.drivetrain.setControl(RobotContainer.robotDrive.withRotationalRate(0).withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
