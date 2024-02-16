// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  // Create our two cameras for each side of the robot
  private final PhotonCamera leftCamera, rightCamera;
  private final Transform3d robotToLeftCam = new Transform3d(new Translation3d(-0.5, 0.5, 0), new Rotation3d(0,0,0)), robotToRightCam = new Transform3d(new Translation3d(0.5, 0.5, 0), new Rotation3d(0,0,0));
  private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final PhotonPoseEstimator leftEstimator, rightEstimator;
  private VisionSystemSim visionSimulator;
  private PhotonCameraSim leftCameraSimulator, rightCameraSimulator;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    leftCamera = new PhotonCamera("Left Camera");
    rightCamera = new PhotonCamera("Right Camera");

    leftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, robotToLeftCam);
    rightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, robotToRightCam);
    leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (Robot.isSimulation()) {
      visionSimulator = new VisionSystemSim("main");
      visionSimulator.addAprilTags(fieldLayout);
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      leftCameraSimulator = new PhotonCameraSim(leftCamera, cameraProp);
      rightCameraSimulator = new PhotonCameraSim(rightCamera, cameraProp);
      visionSimulator.addCamera(leftCameraSimulator, robotToLeftCam);
      visionSimulator.addCamera(rightCameraSimulator, robotToRightCam);
    }
  }

  public PhotonPipelineResult getLatestLeftResult() {
    return leftCamera.getLatestResult();
  }

  public PhotonPipelineResult getLatestRightResult() {
    return rightCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedLeftPose() {
    return leftEstimator.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedRightPose() {
    return rightEstimator.update();
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      visionSimulator.update(RobotContainer.drivetrain.getState().Pose);
      SmartDashboard.putData("Field", visionSimulator.getDebugField());
    }
  }
}
