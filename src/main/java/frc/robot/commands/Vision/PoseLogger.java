// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PoseLogger extends Command {
  private final Field2d leftField, rightField;
  private Optional<EstimatedRobotPose> estimatedLeftPose, estimatedRightPose;
  
  /** Creates a new PoseLogger. */
  public PoseLogger() {
    leftField = new Field2d();
    rightField = new Field2d();
    SmartDashboard.putData("VisionSubsystem/Left Camera Field", leftField);
    SmartDashboard.putData("VisionSubsystem/Right Camera Field", rightField);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    estimatedLeftPose = RobotContainer.vision.getEstimatedLeftPose();
    estimatedRightPose = RobotContainer.vision.getEstimatedRightPose();

    estimatedLeftPose.ifPresent(leftPose -> {
      leftField.setRobotPose(leftPose.estimatedPose.toPose2d());
      SmartDashboard.putData("VisionSubsystem/Left Camera Field", leftField);
    });
    estimatedRightPose.ifPresent(rightPose -> {
      rightField.setRobotPose(rightPose.estimatedPose.toPose2d());
      SmartDashboard.putData("VisionSubsystem/Right Camera Field", rightField);
    });
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
