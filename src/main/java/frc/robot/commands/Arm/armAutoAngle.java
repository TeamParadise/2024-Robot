// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class armAutoAngle extends Command {
  /** Creates a new armAutoAngle. */
  double setpoint, output, positionDegrees, positionRadians, velocity;

  private final double 
    kp = .25, 
    ki = 0, 
    kd = 0,
    maxAccel = .5,
    maxVelo = 1;

    /*feedforwardMax is the max volts that need to be supplied to match gravity (arm at 0 degrees)*/
  public double feedforwardMax = 0;
  PIDController armController = new PIDController(kp, ki, kd);
  
  public armAutoAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = RobotContainer.m_ArmSubsystem.findOptimalAngle();
    positionDegrees = RobotContainer.m_intakeSubsystem.getArmPosition();
    positionRadians = Math.toRadians(Math.toRadians(positionDegrees));
    output = armController.calculate(positionDegrees, setpoint) + feedforwardMax*Math.cos(positionRadians);
    System.out.println(RobotContainer.m_ArmSubsystem.findOptimalAngle());
    SmartDashboard.putNumber("autoArmError", armController.getPositionError());
    SmartDashboard.putNumber("feedforward", feedforwardMax*Math.cos(positionRadians));
    SmartDashboard.putNumber("preClampVoltage", output);

    
    RobotContainer.m_ArmSubsystem.setVoltage(MathUtil.clamp(output, -5, 5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ArmSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
