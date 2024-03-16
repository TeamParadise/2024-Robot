// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Vision.PoseLogger;
import frc.robot.commands.Vision.SingleTagVisionPoseEstimator;
import frc.robot.commands.Vision.VisionPoseEstimator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmLUT;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static ArmLUT m_ArmLUTAngle;
  public static ArmLUT m_ArmLUTRPM;

  static PathPlannerAuto centerSpeakerThreeNote, centerSpeakerTwoNote, centerSpeakerOneNote, leftSpeakerOneNote, amp, leave, none, rightSpeakerOneNote, rightSpeakerTwoNote, rightSpeakerThreeNote;
  
  public static String previouslySelectedAuto = "Left", currentlySelectedAutoSD = "Left", allianceCurrentlySelectedAuto = "Left";

  @Override
  public void robotInit() {
    initLUT();
    m_robotContainer = new RobotContainer();
    RobotContainer.drivetrain.configurePathPlanner(true);
    centerSpeakerThreeNote = new PathPlannerAuto("Speaker Center 3 Note Fast");
    centerSpeakerTwoNote = new PathPlannerAuto("Speaker Center 2 Note Fast");
    centerSpeakerOneNote = new PathPlannerAuto("Speaker Center 1 Note");
    leftSpeakerOneNote = new PathPlannerAuto("Speaker Left 1 Note");
    amp = new PathPlannerAuto("Amp Defense");
    leave = new PathPlannerAuto("Leave");
    none = new PathPlannerAuto("None");
    rightSpeakerOneNote = new PathPlannerAuto("Speaker Right 1 Note");
    rightSpeakerTwoNote = new PathPlannerAuto("Speaker Right 2 Note");
    rightSpeakerThreeNote = new PathPlannerAuto("Speaker Right 3 Note");

    m_autonomousCommand = new PathPlannerAuto("None");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Distance", RobotContainer.m_ArmSubsystem.getDistance());
 
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
      RobotContainer.drivetrain.setDefaultCommand(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-RobotContainer.joystick.getLeftY() * RobotContainer.MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-RobotContainer.joystick.getLeftX() * RobotContainer.MaxSpeed) // Drive left with negative X (left)
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

  private void initLUT() {
    m_ArmLUTAngle = new ArmLUT<>();
    m_ArmLUTRPM = new ArmLUT<>();

    m_ArmLUTAngle.put(1.455, 59.05);
    m_ArmLUTRPM.put(1.455, 2000);
    
    m_ArmLUTAngle.put(1.75, 51.00);
    m_ArmLUTRPM.put(1.75, 2204);

    m_ArmLUTAngle.put(2.00, 48.01);
    m_ArmLUTRPM.put(2.00, 2409);
    
    m_ArmLUTAngle.put(2.31, 45.03);
    m_ArmLUTRPM.put(2.31, 2701);
    
    m_ArmLUTAngle.put(2.61, 39.01);
    m_ArmLUTRPM.put(2.61, 3251);
    
    m_ArmLUTAngle.put(2.91, 35.50); 
    m_ArmLUTRPM.put(2.91, 3500);
    
    m_ArmLUTAngle.put(3.25, 33.50);
    m_ArmLUTRPM.put(3.25, 3600); //3750

    m_ArmLUTAngle.put(3.56, 33.00); //34.04
    m_ArmLUTRPM.put(3.56, 3600);

    m_ArmLUTAngle.put(3.78, 32.00); //34.04
    m_ArmLUTRPM.put(3.78, 3600);

    m_ArmLUTAngle.put(4.04, 31.30); //34.04
    m_ArmLUTRPM.put(4.04, 3600);
    
    m_ArmLUTAngle.put(4.45, 30.00); //34.04
    m_ArmLUTRPM.put(4.45, 3600);

    /*m_ArmLUTAngle.put(2.06, 47.2);
    m_ArmLUTRPM.put(2.06, 2500);
    
    m_ArmLUTAngle.put(3.086, 42.19);
    m_ArmLUTRPM.put(3.086, 2750);
    
    m_ArmLUTAngle.put(4.12, 40.2);
    m_ArmLUTRPM.put(4.12, 3000);
    
    m_ArmLUTAngle.put(5.04, 37.36);
    m_ArmLUTRPM.put(5.04, 3850);
    
    m_ArmLUTAngle.put(6.121, 36.72);
    m_ArmLUTRPM.put(6.121, 4250);*/
  }
}
