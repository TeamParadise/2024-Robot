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

public class alignNoteDriveReq extends Command {
  PhotonPipelineResult visionResult;
  PIDController turnController;
  double tx, lastTimeDetected, timeSinceNote, velocity, originalDetection;
  
  /** Creates a new allignNote. */
  public  alignNoteDriveReq(double speed) {
    addRequirements(RobotContainer.drivetrain);
    this.velocity = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceNote = -1;
    originalDetection = 0;
    turnController = new PIDController(0.18, 0.0, 0);
    turnController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionResult = RobotContainer.vision.intakeCamera.getLatestResult();
    if (visionResult.hasTargets()) {
      // lastTimeDetected = RobotController.getFPGATime();
      RobotContainer.drivetrain.setControl(RobotContainer.robotDrive.withRotationalRate(turnController.calculate(visionResult.getBestTarget().getYaw(), 0)).withVelocityX(velocity));
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
    // if (RobotContainer.m_primerSubsystem.getPrimerBeamBreaker()) {
    //   return true;
    // }
    // return false;
  //   if (RobotContainer.m_primerSubsystem.getPrimerBeamBreaker() && timeSinceNote == -1) {
  //     originalDetection = RobotController.getFPGATime();
  //     timeSinceNote = 0;
  //   } else if (RobotContainer.m_primerSubsystem.getPrimerBeamBreaker()) {
  //     timeSinceNote = RobotController.getFPGATime() - originalDetection;
  //   } else {
  //     originalDetection = 0;
  //     timeSinceNote = -1;
  //   }

  //   if (timeSinceNote > 350000) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }
  return RobotContainer.m_primerSubsystem.getPrimerBeamBreaker();
  }
}
