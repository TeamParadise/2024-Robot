// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Vision.PoseLogger;
import frc.robot.commands.Vision.SingleTagVisionPoseEstimator;
import frc.robot.commands.Vision.VisionPoseEstimator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  
  public static String previouslySelectedAuto = "Left", currentlySelectedAutoSD = "Left", allianceCurrentlySelectedAuto = "Left";

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    RobotContainer.drivetrain.configurePathPlanner(false);
    m_autonomousCommand = new PathPlannerAuto("None");
  }

  @Override
  public void disabledPeriodic() {
    currentlySelectedAutoSD = RobotContainer.mainAutoChooser.getSelected();
    
    if ((DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue)) && currentlySelectedAutoSD == "Left") || (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red)) && currentlySelectedAutoSD == "Right")) {
      SmartDashboard.putData("Auto Task Chooser", RobotContainer.leftAutoChooser);
      allianceCurrentlySelectedAuto = "Left";
    } else if (currentlySelectedAutoSD == "Center") {
      SmartDashboard.putData("Auto Task Chooser", RobotContainer.centerAutoChooser);
      allianceCurrentlySelectedAuto = currentlySelectedAutoSD;
    } else {
      SmartDashboard.putData("Auto Task Chooser", RobotContainer.rightAutoChooser);
      allianceCurrentlySelectedAuto = "Right";
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    RobotContainer.drivetrain.configurePathPlanner(DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red)) ? true : false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      if (Constants.VisionConstants.kVisionEnabled) {
        RobotContainer.vision.setDefaultCommand(Constants.VisionConstants.kExtraVisionDebug ? new SingleTagVisionPoseEstimator().alongWith(new PoseLogger()) : new SingleTagVisionPoseEstimator());
      }
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (Constants.VisionConstants.kVisionEnabled) {
      RobotContainer.vision.setDefaultCommand(Constants.VisionConstants.kExtraVisionDebug ? new VisionPoseEstimator().alongWith(new PoseLogger()) : new VisionPoseEstimator());
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    if (Constants.VisionConstants.kVisionEnabled) {
      RobotContainer.vision.getVisionSimulator().ifPresent(visionSimulator -> visionSimulator.update(RobotContainer.drivetrain.getState().Pose));
    }
  }
}
