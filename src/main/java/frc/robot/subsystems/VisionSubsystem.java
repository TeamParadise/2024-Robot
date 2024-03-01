// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  public final PhotonCamera leftCamera, rightCamera, intakeCamera;
  private final PhotonPoseEstimator leftEstimator, rightEstimator;
  private VisionSystemSim visionSimulator;
  private TargetModel target = new TargetModel(0.3556, 0.3556), target2 = new TargetModel(0.3556, 0.3556), target3 = new TargetModel(0.3556, 0.3556), target4 = new TargetModel(0.3556, 0.3556), target5 = new TargetModel(0.3556, 0.3556);
  private VisionTargetSim targetSim1 = new VisionTargetSim(new Pose3d(8.232666, 7.506370, 0, new Rotation3d(0, 0, 0)), target), targetSim2 = new VisionTargetSim(new Pose3d(8.232666, 0.828734, 0, new Rotation3d(0, 0, 0)), target2);
  private PhotonCameraSim leftCameraSimulator, rightCameraSimulator, intakeCameraSimulator;
  private SimCameraProperties cameraSimulatorProperties;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    leftCamera = new PhotonCamera("Left Camera");
    rightCamera = new PhotonCamera("Right Camera");
    intakeCamera = new PhotonCamera("Intake Camera");

    leftEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, VisionConstants.kRobotToLeftCam);
    rightEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, VisionConstants.kRobotToRightCam);
    leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (Robot.isSimulation() && !VisionConstants.kPhysicalSimulation && VisionConstants.kVisionPoseEstimationSimulation) {
      visionSimulator = new VisionSystemSim("main");
      // visionSimulator.addAprilTags(VisionConstants.kFieldLayout);

      cameraSimulatorProperties = new SimCameraProperties();
      cameraSimulatorProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraSimulatorProperties.setCalibError(0.35, 0.10);
      cameraSimulatorProperties.setFPS(40);
      cameraSimulatorProperties.setAvgLatencyMs(50);
      cameraSimulatorProperties.setLatencyStdDevMs(15);

      leftCameraSimulator = new PhotonCameraSim(leftCamera, cameraSimulatorProperties);
      rightCameraSimulator = new PhotonCameraSim(rightCamera, cameraSimulatorProperties);
      intakeCameraSimulator = new PhotonCameraSim(intakeCamera, cameraSimulatorProperties);
      visionSimulator.addCamera(leftCameraSimulator, VisionConstants.kRobotToLeftCam);
      visionSimulator.addCamera(rightCameraSimulator, VisionConstants.kRobotToRightCam);
      visionSimulator.addCamera(intakeCameraSimulator, new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, Units.degreesToRadians(180))));
      visionSimulator.addVisionTargets(targetSim1);
      visionSimulator.addVisionTargets(targetSim2);
    }
  }

  public PhotonPipelineResult getLatestLeftResult() {
    return leftCamera.getLatestResult();
  }

  public PhotonPipelineResult getLatestRightResult() {
    return rightCamera.getLatestResult();
  }

  public Optional<PhotonPipelineResult> getLatestIntakeResult() {
    return Optional.of(intakeCamera.getLatestResult());
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
