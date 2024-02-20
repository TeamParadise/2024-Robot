// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class armPID extends Command {
  /** Creates a new armController. */
  double setpoint, output, positionDegrees, velocity;
  private final double 
                      kp = 0.0045, 
                      ki = 0, 
                      kd = 0;

  PIDController armController = new PIDController(kp, ki, kd);
  
  public armPID(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ArmSubsystem);
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.enableContinuousInput(0, 360);
  }
  


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    positionDegrees = RobotContainer.m_shooterSubsystem.getArmPos();
    output = armController.calculate(positionDegrees, setpoint);
    RobotContainer.m_ArmSubsystem.setSpeed(MathUtil.clamp(output, -0.2, 0.2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ArmSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
