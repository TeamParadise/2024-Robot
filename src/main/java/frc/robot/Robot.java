// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.ArmLUT;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static ArmLUT m_ArmLUTAngle;
  public static ArmLUT m_ArmLUTRPM;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    initLUT();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Distance", RobotContainer.m_ArmSubsystem.getDistance());
 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
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

    m_ArmLUTAngle.put(0, 0);
    m_ArmLUTRPM.put(0, 0);
  }
}
