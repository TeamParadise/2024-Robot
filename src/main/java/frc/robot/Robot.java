// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Vision.PoseLogger;
import frc.robot.commands.Vision.SingleTagVisionPoseEstimator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static Optional<Alliance> currentAlliance = Optional.empty();

  @Override
  public void robotInit() {
    // Get alliance and configure auto
    currentAlliance = DriverStation.getAlliance();
    RobotContainer.drivetrain.configurePathPlanner(currentAlliance.equals(Optional.of(DriverStation.Alliance.Red)) ? true : false);

    // Configure robot container
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // Alliance Flipping for Autos
    if (DriverStation.isDSAttached() && !currentAlliance.equals(DriverStation.getAlliance())) {
      RobotContainer.drivetrain.configurePathPlanner(DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red)) ? true : false);
      RobotContainer.autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", RobotContainer.autoChooser);
      currentAlliance = DriverStation.getAlliance();
    }

    // Sanity checks for before the match starts
    SmartDashboard.putBoolean("Left Camera Alive", RobotContainer.vision.leftCamera.isConnected());
    SmartDashboard.putBoolean("Right Camera Alive", RobotContainer.vision.rightCamera.isConnected());
    SmartDashboard.putBoolean("Intake Camera Alive", RobotContainer.vision.intakeCamera.isConnected());
    SmartDashboard.putBoolean("CTRE CAN Alive", RobotContainer.drivetrain.getModule(2).getDriveMotor().getVersion().getValueAsDouble() != 0);
    SmartDashboard.putBoolean("Autonomous Alliance Color", currentAlliance.equals(Optional.of(DriverStation.Alliance.Red)) ? false : true);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Shuffleboard.selectTab("Match");
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
  public void autonomousExit() {
    RobotContainer.drivetrain.setControl(RobotContainer.robotDrive.withRotationalRate(0).withVelocityX(0).withVelocityY(0));
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      RobotContainer.drivetrain.setDefaultCommand(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.focDrive.withVelocityX(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? RobotContainer.joystick.getLeftY() * RobotContainer.MaxSpeed : -RobotContainer.joystick.getLeftY() * RobotContainer.MaxSpeed) // Drive forward with
        .withVelocityY(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? RobotContainer.joystick.getLeftX() * RobotContainer.MaxSpeed : -RobotContainer.joystick.getLeftX()) // Drive left with negative X (left)
        .withRotationalRate(-RobotContainer.joystick.getRightX() * RobotContainer.MaxAngularRate
      )));
    }

    if (Constants.VisionConstants.kVisionEnabled) {
      RobotContainer.vision.setDefaultCommand(Constants.VisionConstants.kExtraVisionDebug ? new SingleTagVisionPoseEstimator().alongWith(new PoseLogger()) : new SingleTagVisionPoseEstimator());
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
