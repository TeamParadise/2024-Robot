// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class alignNoteTranslation extends Command {
  PhotonPipelineResult visionResult;
  double robotX, robotY;
  PIDController driveController;
  Translation2d translated;
  
  /** Creates a new allignNote. */
  public alignNoteTranslation() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController = new PIDController(0.12, 0.0, 0.03);
    driveController.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionResult = RobotContainer.vision.intakeCamera.getLatestResult();

    if (visionResult.hasTargets()) {
      robotX = 0;
      robotY = -driveController.calculate(visionResult.getBestTarget().getYaw(), 0);

      translated = new Translation2d(robotX, robotY).rotateBy(RobotContainer.drivetrain.getPoseRotation());
      RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(translated.getX()).withVelocityY(translated.getY()));
    } else {
      RobotContainer.drivetrain.setControl(RobotContainer.drive.withRotationalRate(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.drivetrain.setControl(RobotContainer.drive.withRotationalRate(0).withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
