// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class VisionPoseEstimator extends Command {
  private Optional<EstimatedRobotPose> estimatedLeftPose, estimatedRightPose;
  
  /** Creates a new PoseEstimator. */
  public VisionPoseEstimator() {
    addRequirements(RobotContainer.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    estimatedLeftPose = RobotContainer.vision.getEstimatedLeftPose();
    estimatedRightPose = RobotContainer.vision.getEstimatedRightPose();

    if (RobotContainer.vision.getLatestLeftResult().getMultiTagResult().estimatedPose.isPresent) {
      estimatedLeftPose.ifPresent(leftPose -> RobotContainer.drivetrain.addVisionMeasurement(leftPose.estimatedPose.toPose2d(), leftPose.timestampSeconds));
    } else if (RobotContainer.vision.getLatestRightResult().getMultiTagResult().estimatedPose.isPresent) {
      estimatedRightPose.ifPresent(rightPose -> RobotContainer.drivetrain.addVisionMeasurement(rightPose.estimatedPose.toPose2d(), rightPose.timestampSeconds));
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
