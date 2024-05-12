// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    driveController = new PIDController(0.13, 0.0, 0.004 );
    driveController.setTolerance(0);
    driveController.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionResult = RobotContainer.vision.intakeCamera.getLatestResult();

    if (visionResult.hasTargets()) {
      robotX = -2;
      robotY = -driveController.calculate(visionResult.getBestTarget().getYaw(), 0);

      translated = new Translation2d(robotX, robotY).rotateBy(RobotContainer.drivetrain.getPoseRotation());
      RobotContainer.drivetrain.setControl(RobotContainer.headingDrive.withVelocityX(translated.getX()).withVelocityY(translated.getY()).withTargetDirection(new Rotation2d(Math.atan2(5.475 - RobotContainer.drivetrain.getState().Pose.getY(),  (16.5 - Units.inchesToMeters(36.125)) - RobotContainer.drivetrain.getState().Pose.getX()))));
    } else {
      RobotContainer.drivetrain.setControl(RobotContainer.headingDrive.withVelocityX(0));
      System.out.println("not found");
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
