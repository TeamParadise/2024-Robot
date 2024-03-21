// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class armPID extends Command {
  /** Creates a new ArmController. */
  double setpoint, output, positionDegrees, positionRadians, velocity;

  private final double 
    kp = .65, 
    ki = 0, 
    kd = 0,
    maxAccel = .5,
    maxVelo = 1;

    /*feedforwardMax is the max volts that need to be supplied to match gravity (arm at 0 degrees)*/
  public double feedforwardMax = 0;
  PIDController armController = new PIDController(kp, ki, kd);
  
  public armPID(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ArmSubsystem);
    this.setpoint = setpoint;
    SmartDashboard.putNumber("Position", 50);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      positionDegrees = RobotContainer.m_intakeSubsystem.getArmPosition();
      positionRadians = Math.toRadians(Math.toRadians(positionDegrees));


      
      output = armController.calculate(positionDegrees, setpoint) + feedforwardMax*Math.cos(positionRadians);



      SmartDashboard.putNumber("Distance", RobotContainer.m_ArmSubsystem.getDistance());
      SmartDashboard.putNumber("feedforward", feedforwardMax*Math.cos(positionRadians));
      // setpoint = SmartDashboard.getNumber("Position", 10);
      setpoint = MathUtil.clamp(setpoint, 0, 54);
      SmartDashboard.putNumber("Error", armController.getPositionError());

      RobotContainer.m_ArmSubsystem.setVoltage(MathUtil.clamp(output, -7.5, 7.5));
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
