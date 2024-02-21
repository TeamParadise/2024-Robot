// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera leftCamera, rightCamera;
  private final PhotonPoseEstimator leftEstimator, rightEstimator;
  private VisionSystemSim visionSimulator;
  private PhotonCameraSim leftCameraSimulator, rightCameraSimulator;
  private SimCameraProperties cameraSimulatorProperties;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    leftCamera = new PhotonCamera("Left Camera");
    rightCamera = new PhotonCamera("Right Camera");

    leftEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, VisionConstants.kRobotToLeftCam);
    rightEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, VisionConstants.kRobotToRightCam);
    leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (Robot.isSimulation() && !VisionConstants.kPhysicalSimulation && VisionConstants.kVisionPoseEstimationSimulation) {
      visionSimulator = new VisionSystemSim("main");
      visionSimulator.addAprilTags(VisionConstants.kFieldLayout);

      cameraSimulatorProperties = new SimCameraProperties();
      cameraSimulatorProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraSimulatorProperties.setCalibError(0.35, 0.10);
      cameraSimulatorProperties.setFPS(40);
      cameraSimulatorProperties.setAvgLatencyMs(50);
      cameraSimulatorProperties.setLatencyStdDevMs(15);

      leftCameraSimulator = new PhotonCameraSim(leftCamera, cameraSimulatorProperties);
      rightCameraSimulator = new PhotonCameraSim(rightCamera, cameraSimulatorProperties);
      visionSimulator.addCamera(leftCameraSimulator, VisionConstants.kRobotToLeftCam);
      visionSimulator.addCamera(rightCameraSimulator, VisionConstants.kRobotToRightCam);
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
  
  public Optional<VisionSystemSim> getVisionSimulator() {
    return visionSimulator != null ? Optional.of(visionSimulator) : Optional.empty();
  }

  @Override
  public void periodic() {}
}
